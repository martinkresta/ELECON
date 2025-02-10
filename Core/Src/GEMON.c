/*
 * GEMON.c
 *
 *  Created on: 8. 2. 2025
 *      Author: marti
 *      Brief: Global Energy MONitoring
 *             SW module for combining data from 2 LEMONs :-)
 *             - Uses only data from VARs, so it is independent on hardware.
 */



#include "GEMON.h"
#include "COM.h"
#include "RTC.h"


sGemon mGmon;
sElUtil mEutil;

static void SendBalanceInfo(void);

void GEMON_Init(void)
{
  mEutil.ChargeTargetHour = 15;
}

void GEMON_Update_1s(void)
{
  sDateTime now;
  uint16_t invalid = 0;
  float totalCapacity;
  float optChargePower;
  float totalOptBalanceCurrent;

  // calculate

  mGmon.BatEnergy = VAR_GetVariable(VAR_STRG1_ENERGY_WH, &invalid) + VAR_GetVariable(VAR_STRG2_ENERGY_WH, &invalid) ;
 // mGmon.BatVoltage = (VAR_GetVariable(VAR_STRG1_VOLTAGE_V10, &invalid) + VAR_GetVariable(VAR_STRG2_VOLTAGE_V10, &invalid))/20.0 ;
  mGmon.BatVoltage = (VAR_GetVariable(VAR_STRG1_VOLTAGE_V10, &invalid))/10.0 ; // Second storage does not exist yet
  totalCapacity = (VAR_GetVariable(VAR_STRG1_CAPACITY_AH, &invalid) + VAR_GetVariable(VAR_STRG2_CAPACITY_AH, &invalid)) * mGmon.BatVoltage ;
  mGmon.BatPower = VAR_GetVariable(VAR_STRG1_POWER_W, &invalid) + VAR_GetVariable(VAR_STRG2_POWER_W, &invalid) ;
  mGmon.BatCurrent = VAR_GetVariable(VAR_STRG1_CURRENT_A10, &invalid)/10.0 + VAR_GetVariable(VAR_STRG2_CURRENT_A10, &invalid)/10.0 ;
  mGmon.BatSoc =  100 *  mGmon.BatEnergy / totalCapacity;
  mGmon.LoadPower = VAR_GetVariable(VAR_LLOAD1_POWER_W, &invalid) + VAR_GetVariable(VAR_LLOAD2_POWER_W, &invalid) ;
  mGmon.LoadCons_Wh = VAR_GetVariable(VAR_LLOAD1_CONS_WH, &invalid) + VAR_GetVariable(VAR_LLOAD2_CONS_WH, &invalid) ;
  mGmon.SolarPower = VAR_GetVariable(VAR_LPV1_POWER_W, &invalid) + VAR_GetVariable(VAR_LPV2_POWER_W, &invalid) ;
  mGmon.SolarProd_Wh = VAR_GetVariable(VAR_LPV1_PROD_WH, &invalid) + VAR_GetVariable(VAR_LPV2_PROD_WH, &invalid) ;
  totalOptBalanceCurrent = VAR_GetVariable(VAR_STRG1_OPT_CHARGING_A, &invalid) + VAR_GetVariable(VAR_STRG2_OPT_CHARGING_A, &invalid) ;  // calculated by lemon to support balancing
  mEutil.Strg1.BalancedToday =  VAR_GetVariable(VAR_STRG1_BALANCED_TODAY, &invalid);
  mEutil.Strg2.BalancedToday =  VAR_GetVariable(VAR_STRG2_BALANCED_TODAY, &invalid);
  // Balancing and optimal charging support


  // Maximal utilization of available PV energy:
  // calculate optimal charging current to reach full SOC at certain time

  mEutil.now = RTC_GetTime();
  now = mEutil.now;
  if (now.Hour < mEutil.ChargeTargetHour && now.Month > 2 && now.Month < 11)  // Only if it is not a DARK_SEASON from November to February
  {
    mEutil.RemainingTime_s = (mEutil.ChargeTargetHour - now.Hour - 1) * 3600 + (60 - now.Minute)*60;

    optChargePower = (totalCapacity - mGmon.BatEnergy) / (mEutil.RemainingTime_s /3600.0);    // Missing Wh / Remaining hours
    mEutil.OptChargeCurrent = optChargePower / mGmon.BatVoltage;
    mEutil.OptChargeCurrent -= 4;  // pre-compensate quantization error  (10A) by ELHEATER
  }
  else
  {
    mEutil.OptChargeCurrent = 500;
  }

  // use the lower value of calculated optimal charging current
  if(totalOptBalanceCurrent < mEutil.OptChargeCurrent)
  {
    mEutil.OptChargeCurrent = totalOptBalanceCurrent;
  }

  SendBalanceInfo();


  // Publish
  VAR_SetVariable(VAR_BAT_SOC,(int16_t)(mGmon.BatSoc),1);
  VAR_SetVariable(VAR_BAT_VOLTAGE_V10,(int16_t)(mGmon.BatVoltage*10),1);
  VAR_SetVariable(VAR_BAT_CURRENT_A10,(int16_t)(mGmon.BatCurrent*10),1);
  VAR_SetVariable(VAR_BAT_POWER_W,(int16_t)(mGmon.BatPower),1);
  VAR_SetVariable(VAR_BAT_ENERGY_WH,(int16_t)(mGmon.BatEnergy),1);

  VAR_SetVariable(VAR_SOLAR_POWER_W,(int16_t)(mGmon.SolarPower),1);
  VAR_SetVariable(VAR_SOLAR_ENERGY_TODAY_10WH,(int16_t)(mGmon.SolarProd_Wh/10), 1);
  //VAR_SetVariable(VAR_CHARGING_A10,(int16_t)(10 * mGmon.SolarPower / mGmon.BatVoltage),1);  // !!! WARNING Used by TECHM ELHEATER

  VAR_SetVariable(VAR_LOAD_W,(int16_t)(mGmon.LoadPower),1);
  VAR_SetVariable(VAR_CONS_TODAY_WH,(int16_t)(mGmon.LoadCons_Wh), 1);
  VAR_SetVariable(VAR_LOAD_A100,(int16_t)(100 * mGmon.LoadPower / mGmon.BatVoltage),1);  // Auxiliary value just for WEB visualisation

}



static void SendBalanceInfo(void)
{
  // electric heater load control  (to lower charging current)
  uint8_t txdata[8];
  uint8_t balancedToday = 0;
  int16_t OptCurr;

  if(mEutil.Strg1.BalancedToday) // && mEutil.Strg2.BalancedToday)
  {
    balancedToday = 1;
  }

  OptCurr = (int16_t)mEutil.OptChargeCurrent;

  // Send status of balancedtoday and optimal balancing current to TECHM
  txdata[0] = 0;
  txdata[1] = balancedToday;
  txdata[2] = OptCurr >> 8;
  txdata[3] = OptCurr & 0xFF;
  txdata[4] = 0;
  txdata[5] = 0;
  txdata[6] = 0;
  txdata[7] = 0;
  COM_SendMessage(CMD_BALANCE_INFO, txdata, 8);
}
