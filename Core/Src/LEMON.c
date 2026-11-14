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
#include "VARS2.h"


sLemon mLemon;
sBMS mBms80;
sBMS mBms220;

static sVarsLocalEnergy* mpEnergy;   /* produced: Storage, Load, Pv */
static const sVarsMppt* mpMppt;      /* read */


static void SocReinit(void);
static void CalculateOptimalCharging(void);
static void CheckFullyCharged(void);
static void Publish(void);

void LEMON_Init(UART_HandleTypeDef* huart80, UART_HandleTypeDef* huart220, sVarsLocalEnergy* pEnergy,
                sVarsPack* pPack80, sVarsPack* pPack220, const sVarsMppt* pMppt)
{
  mLemon.Bms80 = &mBms80;
  mLemon.Bms220 = &mBms220;
  mpEnergy = pEnergy;
  mpMppt = pMppt;

  // Configuration
  mLemon.Cfg.Pack80_Ah = 80;
  mLemon.Cfg.Pack220_Ah = 220;
  mLemon.Cfg.Shunt_R_uOhm = 250;  // not used so far


  // Init the underlayer
  if(mLemon.Cfg.Pack80_Ah != 0)
  {
    mLemon.Bms80->Huart = huart80;
    BMS_Init(mLemon.Bms80, pPack80);   // the BMS driver publishes the pack group
  }
  if(mLemon.Cfg.Pack220_Ah != 0)
  {
    mLemon.Bms220->Huart = huart220;
    BMS_Init(mLemon.Bms220, pPack220);
  }
  // SHUNT and MPPT should be initialized from APP.c

  // Init the values - Estimate the SOC based on BMS soc
  mLemon.SocInitialized = 0;

}

void LEMON_Update_1s(void)
{
  // Update BMSs
  BMS_Update(mLemon.Bms80);
  BMS_Update(mLemon.Bms220);


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
 // mLemon.Storage.Voltage = (mLemon.Bms80->LiveData.VoltageTotal_mV + mLemon.Bms220->LiveData.VoltageTotal_mV)/2000.0;  // mV to V

  // limit 100%
  if((mLemon.Internal.Avilable_mAs/AH2MAS) > (mLemon.Internal.TotalCapacity_Ah))
  {
    mLemon.Internal.Avilable_mAs = (mLemon.Internal.TotalCapacity_Ah * AH2MAS);
  }

  mLemon.Storage.Voltage = (mLemon.Bms220->LiveData.VoltageTotal_mV)/1000.0;  // mV to V
  mLemon.Storage.Energy = mLemon.Internal.Avilable_mAs * mLemon.Storage.Voltage / AH2MAS;   // Ah * V => Wh
  mLemon.Storage.Power = mLemon.Storage.Current * mLemon.Storage.Voltage;
  mLemon.Storage.Soc = 100 * (mLemon.Internal.Avilable_mAs / AH2MAS) / (mLemon.Internal.TotalCapacity_Ah);  // in percents

  // Production (MPPT validity not checked, as before)
  mLemon.Prod.Power = mpMppt->SolarPower_W.value;
  mLemon.Prod.ChargeCurrent = mpMppt->BatCurrent_A.value;
  mLemon.Prod.Prod_Wh += (mLemon.Prod.Power / 3600.0);  // wS to wH

  // Load
  mLemon.Load.Power = -mLemon.Storage.Power + mLemon.Prod.Power;
  mLemon.Load.Cons_Wh += (mLemon.Load.Power / 3600.0);  // wS to wH
  mLemon.Load.LoadCurrent = mLemon.Storage.Current + mLemon.Prod.ChargeCurrent;


  // Calculate optimal charging power (Only w.r.t to balancing support)
  CalculateOptimalCharging();

  // Publish
  Publish();
}

void LEMON_MidnightNow(void)
{
  mLemon.BalancedTodayFlag = 0;
  mLemon.Prod.Prod_Wh = 0;
  mLemon.Load.Cons_Wh = 0;
}




void LEMON_UartRxCallback(UART_HandleTypeDef *huart, uint16_t reclength)
{
  BMS_UartRxCallback(huart, reclength, mLemon.Bms80, mLemon.Bms220);
}



// ********  Private methods  ********
static void SocReinit(void)
{
  uint8_t ValidInit = 1;
  uint64_t totalAvailable_mAs = 0;

  mLemon.Internal.TotalCapacity_Ah = mLemon.Cfg.Pack80_Ah + mLemon.Cfg.Pack220_Ah;

  if(mLemon.Cfg.Pack80_Ah)   // 80 Ah BMS should communicate
  {
    if(mLemon.Bms80->Active && mLemon.Bms80->LiveData.SOC)
    {
      totalAvailable_mAs += mLemon.Cfg.Pack80_Ah * AH2MAS * mLemon.Bms80->LiveData.SOC / 100;
    }
    else
    {
      ValidInit = 0;
    }
  }

  if(mLemon.Cfg.Pack220_Ah)   // 220 Ah BMS should communicate
  {
    if(mLemon.Bms220->Active && mLemon.Bms220->LiveData.SOC)
    {
      totalAvailable_mAs += mLemon.Cfg.Pack220_Ah * AH2MAS * mLemon.Bms220->LiveData.SOC / 100;
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
  if((!BMS_IsChargingEnabled(mLemon.Bms80) || !BMS_IsChargingEnabled(mLemon.Bms220)) &&
    (mLemon.Bms80->LiveData.SOC >= 99 || mLemon.Bms220->LiveData.SOC >= 99) && !mLemon.ChargingDisabledFlag)
  {
    mLemon.Internal.Avilable_mAs = mLemon.Internal.TotalCapacity_Ah * AH2MAS;  // fully charged
    mLemon.ChargingDisabledFlag = 1;
  }

  if((BMS_IsChargingEnabled(mLemon.Bms80) && BMS_IsChargingEnabled(mLemon.Bms220)))  // Just resetting the flag
  {
    mLemon.ChargingDisabledFlag = 0;
  }


  // Get the minimal cell voltage (only from BMSs which are active)
  mLemon.Internal.MinCellVoltage_mV = 0;
  if(mLemon.Bms80->Active)
  {
    mLemon.Internal.MinCellVoltage_mV = mLemon.Bms80->LiveData.MinCellVoltage_mV;
  }
  if(mLemon.Bms220->Active && (mLemon.Bms220->LiveData.MinCellVoltage_mV < mLemon.Internal.MinCellVoltage_mV))
  {
    mLemon.Internal.MinCellVoltage_mV = mLemon.Bms220->LiveData.MinCellVoltage_mV;
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
  if (mLemon.Bms80->LiveData.MaxCellVoltage_mV > mLemon.Bms220->LiveData.MaxCellVoltage_mV)
  {
    mLemon.Internal.MaxCellVoltage_mV = mLemon.Bms80->LiveData.MaxCellVoltage_mV;
  }
  else
  {
    mLemon.Internal.MaxCellVoltage_mV = mLemon.Bms220->LiveData.MaxCellVoltage_mV;
  }
  if ( mLemon.Internal.MaxCellVoltage_mV >= CELL_BALANCE_MV)
  {
    mLemon.Storage.OptChargingCurrent = 2 + ((CELL_MAX_MV - mLemon.Internal.MaxCellVoltage_mV) * BalanceSupportFactor);  // equation set by experiments
  }
}


// HomeEnergy: always valid (as before: this node has no storage validity)
static void Publish(void)
{
  sVarsLocalEnergy* e = mpEnergy;

  e->Storage.Soc_pct.value = mLemon.Storage.Soc;
  e->Storage.Power_W.value = mLemon.Storage.Power;
  e->Storage.Current_A.value = mLemon.Storage.Current;
  e->Storage.Voltage_V.value = mLemon.Storage.Voltage;
  e->Storage.Energy_Wh.value = mLemon.Storage.Energy;
  e->Storage.OptCharging_A.value = mLemon.Storage.OptChargingCurrent;
  e->Storage.Capacity_Ah.value = mLemon.Internal.TotalCapacity_Ah;
  e->Storage.BalancedToday.value = mLemon.BalancedTodayFlag;
  e->Load.Power_W.value = mLemon.Load.Power;
  e->Load.Cons_Wh.value = mLemon.Load.Cons_Wh;
  e->Pv.Power_W.value = mLemon.Prod.Power;
  e->Pv.Prod_Wh.value = mLemon.Prod.Prod_Wh;
  VARS2_SetGroupValid(e, 1);
}
