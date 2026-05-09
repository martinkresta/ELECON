/*
 * LEMON.c
 *
 *  Created on: 7. 2. 2025
 *      Author: marti
 *      Brief: Local Energy MONitoring
 *             SW module for monitoring local Storage, PV (production) and Load (consumption)
 *             - Uses Drivers for accessing BMS, Shunt, MPPT and ACDC inverter
 */

#include "LEMON.h"
#include "SHUNT.h"
#include "MPPT.h"
#include "BMS.h"
#include "VARS.h"


sLemon mLemon;
sBMS mBms1;
sBMS mBms2;


static void SocReinit(void);
static void CalculateOptimalCharging(void);
static void CheckFullyCharged(void);
static void PublishVARs(void);

void LEMON_Init(UART_HandleTypeDef* huart1, UART_HandleTypeDef* huart2)
{
  mLemon.Bms1 = &mBms1;
  mLemon.Bms2 = &mBms2;

  // Configuration
  mLemon.Cfg.Pack1_Ah = 80;
  mLemon.Cfg.Pack2_Ah = 220;
  mLemon.Cfg.Shunt_R_uOhm = 250;  // not used so far


  // Init the underlayer
  if(mLemon.Cfg.Pack1_Ah != 0)
  {
    mLemon.Bms1->Huart = huart1;
    BMS_Init(mLemon.Bms1);
  }
  if(mLemon.Cfg.Pack2_Ah != 0)
  {
    mLemon.Bms2->Huart = huart2;
    BMS_Init(mLemon.Bms2);
  }
  // SHUNT and MPPT should be initialized from APP.c

  // Init the values - Estimate the SOC based on BMS soc
  mLemon.SocInitialized = 0;

}

void LEMON_Update_1s(void)
{
  uint16_t invalid = 0;
  // Update BMSs
  BMS_Update(mLemon.Bms1);
  BMS_Update(mLemon.Bms2);


  // Check if SOC initialization is needed
  if(!mLemon.SocInitialized)
  {
    SocReinit();
  }

  // check if storage is full
  CheckFullyCharged();


  // Calculate
  // Storage
  mLemon.Storage.Current = (float)(SHUNT_GetIbat_mA() / 1000.0);
  mLemon.Internal.Avilable_mAs += mLemon.Storage.Current * 1000.0;
 // mLemon.Storage.Voltage = (mLemon.Bms1->LiveData.VoltageTotal_mV + mLemon.Bms2->LiveData.VoltageTotal_mV)/2000.0;  // mV to V

  // limit 100%
  if((mLemon.Internal.Avilable_mAs/AH2MAS) > (mLemon.Internal.TotalCapacity_Ah))
  {
    mLemon.Internal.Avilable_mAs = (mLemon.Internal.TotalCapacity_Ah * AH2MAS);
  }

  mLemon.Storage.Voltage = (mLemon.Bms2->LiveData.VoltageTotal_mV)/1000.0;  // mV to V
  mLemon.Storage.Energy = mLemon.Internal.Avilable_mAs * mLemon.Storage.Voltage / AH2MAS;   // Ah * V => Wh
  mLemon.Storage.Power = mLemon.Storage.Current * mLemon.Storage.Voltage;
  mLemon.Storage.Soc = 100 * (mLemon.Internal.Avilable_mAs / AH2MAS) / (mLemon.Internal.TotalCapacity_Ah);  // in percents

  // Production
  mLemon.Prod.Power = VAR_GetVariable(VAR_MPPT_SOLAR_POWER_W, &invalid);
  mLemon.Prod.ChargeCurrent = VAR_GetVariable(VAR_MPPT_BAT_CURRENT_A10, &invalid) / 10.0;
  mLemon.Prod.Prod_Wh += (mLemon.Prod.Power / 3600.0);  // wS to wH

  // Load
  mLemon.Load.Power = -mLemon.Storage.Power + mLemon.Prod.Power;
  mLemon.Load.Cons_Wh += (mLemon.Load.Power / 3600.0);  // wS to wH
  mLemon.Load.LoadCurrent = mLemon.Storage.Current + mLemon.Prod.ChargeCurrent;


  // Calculate optimal charging power (Only w.r.t to balancing support)
  CalculateOptimalCharging();

  // Publish
  PublishVARs();
}

void LEMON_MidnightNow(void)
{
  mLemon.BalancedTodayFlag = 0;
  mLemon.Prod.Prod_Wh = 0;
  mLemon.Load.Cons_Wh = 0;
}




void LEMON_UartRxCallback(UART_HandleTypeDef *huart, uint16_t reclength)
{
  BMS_UartRxCallback(huart, reclength, mLemon.Bms1, mLemon.Bms2);
}



// ********  Private methods  ********
static void SocReinit(void)
{
  uint8_t ValidInit = 1;
  uint64_t totalAvailable_mAs = 0;

  mLemon.Internal.TotalCapacity_Ah = mLemon.Cfg.Pack1_Ah + mLemon.Cfg.Pack2_Ah;

  if(mLemon.Cfg.Pack1_Ah)   // BMS1 should communicate
  {
    if(mLemon.Bms1->Active && mLemon.Bms1->LiveData.SOC)
    {
      totalAvailable_mAs += mLemon.Cfg.Pack1_Ah * AH2MAS * mLemon.Bms1->LiveData.SOC / 100;
    }
    else
    {
      ValidInit = 0;
    }
  }

  if(mLemon.Cfg.Pack2_Ah)   // BMS2 should communicate
  {
    if(mLemon.Bms2->Active && mLemon.Bms2->LiveData.SOC)
    {
      totalAvailable_mAs += mLemon.Cfg.Pack2_Ah * AH2MAS * mLemon.Bms2->LiveData.SOC / 100;
    }
    else
    {
      ValidInit = 0;
    }
  }

  if(ValidInit)
  {

    mLemon.Internal.Avilable_mAs = totalAvailable_mAs;
    mLemon.Storage.Soc = 100 * (mLemon.Internal.Avilable_mAs / AH2MAS) / (mLemon.Internal.TotalCapacity_Ah);  // in percents
    mLemon.SocInitialized = 1;
  }
}


static void CheckFullyCharged(void)
{

  // Check if storage is fully charged (This should happen only once when charging is stopped, thus we need Charging Disabled Flag
  if((!BMS_IsChargingEnabled(mLemon.Bms1) || !BMS_IsChargingEnabled(mLemon.Bms2)) &&
    (mLemon.Bms1->LiveData.SOC >= 99 || mLemon.Bms2->LiveData.SOC >= 99) && !mLemon.ChargingDisabledFlag)
  {
    mLemon.Internal.Avilable_mAs = mLemon.Internal.TotalCapacity_Ah * AH2MAS;  // fully charged
    mLemon.ChargingDisabledFlag = 1;
  }

  if((BMS_IsChargingEnabled(mLemon.Bms1) && BMS_IsChargingEnabled(mLemon.Bms2)))  // Just resetting the flag
  {
    mLemon.ChargingDisabledFlag = 0;
  }


  // Get the minimal cell voltage (only from BMSs which are active)
  mLemon.Internal.MinCellVoltage_mV = 0;
  if(mLemon.Bms1->Active)
  {
    mLemon.Internal.MinCellVoltage_mV = mLemon.Bms1->LiveData.MinCellVoltage_mV;
  }
  if(mLemon.Bms2->Active && (mLemon.Bms2->LiveData.MinCellVoltage_mV < mLemon.Internal.MinCellVoltage_mV))
  {
    mLemon.Internal.MinCellVoltage_mV = mLemon.Bms2->LiveData.MinCellVoltage_mV;
  }


// TBD!   What is the target voltage when everythink is balanced ?
  if (mLemon.Internal.MinCellVoltage_mV >= CELL_BALANCE_MV)  // All cells reached minimal voltage -> balanced today
  {
    mLemon.BalancedTodayFlag = 1;
    mLemon.Internal.Avilable_mAs = mLemon.Internal.TotalCapacity_Ah * AH2MAS;  // fully charged
  }
}

static void CalculateOptimalCharging(void)
{
  float BalanceSupportFactor = (mLemon.Internal.TotalCapacity_Ah / 10)/(CELL_MAX_MV - CELL_BALANCE_MV);  // factor for reducing charging current
  mLemon.Storage.OptChargingCurrent = mLemon.Internal.TotalCapacity_Ah / 2;  // Max charging current = 0.5C


  // Support the cell top balancing by lowering charging current
  if (mLemon.Bms1->LiveData.MaxCellVoltage_mV > mLemon.Bms2->LiveData.MaxCellVoltage_mV)
  {
    mLemon.Internal.MaxCellVoltage_mV = mLemon.Bms1->LiveData.MaxCellVoltage_mV;
  }
  else
  {
    mLemon.Internal.MaxCellVoltage_mV = mLemon.Bms2->LiveData.MaxCellVoltage_mV;
  }
  if ( mLemon.Internal.MaxCellVoltage_mV >= CELL_BALANCE_MV)
  {
    mLemon.Storage.OptChargingCurrent = 2 + ((CELL_MAX_MV - mLemon.Internal.MaxCellVoltage_mV) * BalanceSupportFactor);  // equation set by experiments
  }
}


static void PublishVARs(void)
{
  uint8_t validflag = 1;

  VAR_SetVariable(VAR_STRG1_SOC,(int16_t)(mLemon.Storage.Soc * 100), validflag);
  VAR_SetVariable(VAR_STRG1_POWER_W, (int16_t)mLemon.Storage.Power, validflag);
  VAR_SetVariable(VAR_STRG1_CURRENT_A10, (int16_t)(mLemon.Storage.Current * 10), validflag);
  VAR_SetVariable(VAR_STRG1_VOLTAGE_V10, (int16_t)(mLemon.Storage.Voltage * 10), validflag);
  VAR_SetVariable(VAR_STRG1_ENERGY_WH, (int16_t)mLemon.Storage.Energy, validflag);
  VAR_SetVariable(VAR_STRG1_OPT_CHARGING_A , (int16_t)mLemon.Storage.OptChargingCurrent, validflag);
  VAR_SetVariable(VAR_STRG1_CAPACITY_AH, (int16_t)mLemon.Internal.TotalCapacity_Ah, validflag);
  VAR_SetVariable(VAR_STRG1_BALANCED_TODAY, (int16_t)mLemon.BalancedTodayFlag, validflag);
  VAR_SetVariable(VAR_LLOAD1_POWER_W, (int16_t)mLemon.Load.Power, validflag);
  VAR_SetVariable(VAR_LLOAD1_CONS_WH, (int16_t)mLemon.Load.Cons_Wh, validflag);
  VAR_SetVariable(VAR_LPV1_POWER_W, (int16_t)mLemon.Prod.Power, validflag);
  VAR_SetVariable(VAR_LPV1_PROD_WH, (int16_t)mLemon.Prod.Prod_Wh, validflag);

 /* VAR_SetVariable(VAR_LPV1_S1_POWER_W  311
  VAR_SetVariable(VAR_LPV1_S1_CURRENT_A10  312
  VAR_SetVariable(VAR_LPV1_S1_VOLTAGE_V  313*/

  // Publish BMS1
  if(mLemon.Bms1->Active)
  {
     VAR_SetVariable(VAR_BMS1_VOLTAGE_V10, mLemon.Bms1->LiveData.VoltageTotal_mV/100, validflag);
     VAR_SetVariable(VAR_BMS1_SOC, mLemon.Bms1->LiveData.SOC, validflag);
     VAR_SetVariable(VAR_BMS1_CURRENT_A10, mLemon.Bms1->LiveData.BatteryCurrent_mA/100, validflag);
     VAR_SetVariable(VAR_BMS1_ENERGY_STORED_WH, mLemon.Bms1->LiveData.Energystored_Wh, validflag);
     VAR_SetVariable(VAR_BMS1_TODAY_ENERGY_WH, mLemon.Bms1->LiveData.TodayCharging_Wh, validflag);

     VAR_SetVariable(VAR_BMS1_CELL1_MV, mLemon.Bms1->Cells[0].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL2_MV, mLemon.Bms1->Cells[1].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL3_MV, mLemon.Bms1->Cells[2].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL4_MV, mLemon.Bms1->Cells[3].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL5_MV, mLemon.Bms1->Cells[4].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL6_MV, mLemon.Bms1->Cells[5].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL7_MV, mLemon.Bms1->Cells[6].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL8_MV, mLemon.Bms1->Cells[7].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL9_MV, mLemon.Bms1->Cells[8].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL10_MV, mLemon.Bms1->Cells[9].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL11_MV, mLemon.Bms1->Cells[10].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL12_MV, mLemon.Bms1->Cells[11].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL13_MV, mLemon.Bms1->Cells[12].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL14_MV, mLemon.Bms1->Cells[13].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL15_MV, mLemon.Bms1->Cells[14].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL16_MV, mLemon.Bms1->Cells[15].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS1_CELL1_C, mLemon.Bms1->Cells[0].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL2_C, mLemon.Bms1->Cells[1].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL3_C, mLemon.Bms1->Cells[2].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL4_C, mLemon.Bms1->Cells[3].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL5_C, mLemon.Bms1->Cells[4].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL6_C, mLemon.Bms1->Cells[5].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL7_C, mLemon.Bms1->Cells[6].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL8_C, mLemon.Bms1->Cells[7].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL9_C, mLemon.Bms1->Cells[8].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL10_C,mLemon.Bms1-> Cells[9].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL11_C, mLemon.Bms1->Cells[10].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL12_C, mLemon.Bms1->Cells[11].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL13_C, mLemon.Bms1->Cells[12].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL14_C, mLemon.Bms1->Cells[13].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL15_C, mLemon.Bms1->Cells[14].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS1_CELL16_C, mLemon.Bms1->Cells[15].Temp_C, validflag);
  }

  // Publish BMS2

  if(mLemon.Bms2->Active)
  {
     VAR_SetVariable(VAR_BMS2_VOLTAGE_V10, mLemon.Bms2->LiveData.VoltageTotal_mV/100, validflag);
     VAR_SetVariable(VAR_BMS2_SOC, mLemon.Bms2->LiveData.SOC, validflag);
     VAR_SetVariable(VAR_BMS2_CURRENT_A10, mLemon.Bms2->LiveData.BatteryCurrent_mA/100, validflag);
     VAR_SetVariable(VAR_BMS2_ENERGY_STORED_WH, mLemon.Bms2->LiveData.Energystored_Wh, validflag);
     VAR_SetVariable(VAR_BMS2_TODAY_ENERGY_WH, mLemon.Bms2->LiveData.TodayCharging_Wh, validflag);

     VAR_SetVariable(VAR_BMS2_CELL1_MV, mLemon.Bms2->Cells[0].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL2_MV, mLemon.Bms2->Cells[1].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL3_MV, mLemon.Bms2->Cells[2].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL4_MV, mLemon.Bms2->Cells[3].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL5_MV, mLemon.Bms2->Cells[4].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL6_MV, mLemon.Bms2->Cells[5].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL7_MV, mLemon.Bms2->Cells[6].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL8_MV, mLemon.Bms2->Cells[7].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL9_MV, mLemon.Bms2->Cells[8].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL10_MV, mLemon.Bms2->Cells[9].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL11_MV, mLemon.Bms2->Cells[10].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL12_MV, mLemon.Bms2->Cells[11].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL13_MV, mLemon.Bms2->Cells[12].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL14_MV, mLemon.Bms2->Cells[13].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL15_MV, mLemon.Bms2->Cells[14].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL16_MV, mLemon.Bms2->Cells[15].Voltage_mV, validflag);
     VAR_SetVariable(VAR_BMS2_CELL1_C, mLemon.Bms2->Cells[0].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL2_C, mLemon.Bms2->Cells[1].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL3_C, mLemon.Bms2->Cells[2].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL4_C, mLemon.Bms2->Cells[3].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL5_C, mLemon.Bms2->Cells[4].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL6_C, mLemon.Bms2->Cells[5].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL7_C, mLemon.Bms2->Cells[6].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL8_C, mLemon.Bms2->Cells[7].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL9_C, mLemon.Bms2->Cells[8].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL10_C,mLemon.Bms2-> Cells[9].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL11_C, mLemon.Bms2->Cells[10].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL12_C, mLemon.Bms2->Cells[11].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL13_C, mLemon.Bms2->Cells[12].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL14_C, mLemon.Bms2->Cells[13].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL15_C, mLemon.Bms2->Cells[14].Temp_C, validflag);
     VAR_SetVariable(VAR_BMS2_CELL16_C, mLemon.Bms2->Cells[15].Temp_C, validflag);
  }

}
