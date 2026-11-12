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

// Validity of the published output groups (1 = last computation used only valid inputs).
// While a group is invalid, mGmon keeps its last valid values and they are published with valid = 0.
static uint8_t mBatValid;
static uint8_t mSolarValid;
static uint8_t mLoadValid;
static uint8_t mLoadA100Valid;
static float   mTotalCapacity;   // last valid total capacity [Wh] (used by the optimal charging calculation)
static int16_t mLoadA100;        // last valid VAR_LOAD_A100

static void SendBalanceInfo(void);

void GEMON_Init(void)
{
  mEutil.ChargeTargetHour = 15;
}

void GEMON_Update_1s(void)
{
  sDateTime now;
  uint16_t invBat = 0;
  uint16_t invSolar = 0;
  uint16_t invLoad = 0;
  float totalCapacity;
  float optChargePower;
  float totalOptBalanceCurrent;
  float batEnergy, batVoltage, batPower, batCurrent;
  float loadPower, loadCons_Wh;
  float solarPower, solarProd_Wh;

  // calculate

  // battery group
  batEnergy = VAR_GetVariable(VAR_STRG1_ENERGY_WH, &invBat) + VAR_GetVariable(VAR_STRG2_ENERGY_WH, &invBat) ;
  batVoltage = (VAR_GetVariable(VAR_STRG1_VOLTAGE_V10, &invBat) + VAR_GetVariable(VAR_STRG2_VOLTAGE_V10, &invBat))/20.0 ;
  totalCapacity = (VAR_GetVariable(VAR_STRG1_CAPACITY_AH, &invBat) + VAR_GetVariable(VAR_STRG2_CAPACITY_AH, &invBat)) * batVoltage ;
  batPower = VAR_GetVariable(VAR_STRG1_POWER_W, &invBat) + VAR_GetVariable(VAR_STRG2_POWER_W, &invBat) ;
  batCurrent = (VAR_GetVariable(VAR_STRG1_CURRENT_A10, &invBat)/10.0) + (VAR_GetVariable(VAR_STRG2_CURRENT_A10, &invBat)/10.0) ;
  if (invBat == 0 && totalCapacity > 0 && batVoltage > 0)
  {
    mGmon.BatEnergy = batEnergy;
    mGmon.BatVoltage = batVoltage;
    mGmon.BatPower = batPower;
    mGmon.BatCurrent = batCurrent;
    mGmon.BatSoc =  100 *  mGmon.BatEnergy / totalCapacity;
    mTotalCapacity = totalCapacity;
    mBatValid = 1;
  }
  else
  {
    mBatValid = 0;   // keep last valid battery values
  }
  totalCapacity = mTotalCapacity;

  // load group
  loadPower = VAR_GetVariable(VAR_LLOAD1_POWER_W, &invLoad) + VAR_GetVariable(VAR_LLOAD2_POWER_W, &invLoad) ;
  loadCons_Wh = VAR_GetVariable(VAR_LLOAD1_CONS_WH, &invLoad) + VAR_GetVariable(VAR_LLOAD2_CONS_WH, &invLoad) ;
  if (invLoad == 0)
  {
    mGmon.LoadPower = loadPower;
    mGmon.LoadCons_Wh = loadCons_Wh;
    mLoadValid = 1;
  }
  else
  {
    mLoadValid = 0;
  }
  if (mLoadValid && mBatValid)   // LOAD_A100 also needs the battery voltage (> 0 guaranteed by mBatValid)
  {
    mLoadA100 = (int16_t)(100 * mGmon.LoadPower / mGmon.BatVoltage);
    mLoadA100Valid = 1;
  }
  else
  {
    mLoadA100Valid = 0;
  }

  // solar group
  solarPower = VAR_GetVariable(VAR_LPV1_POWER_W, &invSolar) + VAR_GetVariable(VAR_LPV2_POWER_W, &invSolar) ;
  solarProd_Wh = VAR_GetVariable(VAR_LPV1_PROD_WH, &invSolar) + VAR_GetVariable(VAR_LPV2_PROD_WH, &invSolar) ;
  if (invSolar == 0)
  {
    mGmon.SolarPower = solarPower;
    mGmon.SolarProd_Wh = solarProd_Wh;
    mSolarValid = 1;
  }
  else
  {
    mSolarValid = 0;
  }

  // Balancing and optimal charging support (validity of these inputs is not evaluated, as before)
  totalOptBalanceCurrent = VAR_GetVariable(VAR_STRG1_OPT_CHARGING_A, NULL) + VAR_GetVariable(VAR_STRG2_OPT_CHARGING_A, NULL) ;  // calculated by lemon to support balancing
  mEutil.Strg1.BalancedToday =  VAR_GetVariable(VAR_STRG1_BALANCED_TODAY, NULL);
  mEutil.Strg2.BalancedToday =  VAR_GetVariable(VAR_STRG2_BALANCED_TODAY, NULL);


  // Maximal utilization of available PV energy:
  // calculate optimal charging current to reach full SOC at certain time
  // (uses the last valid battery values)

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


  // Publish (invalid group: last valid values with valid = 0; 0 before the first valid computation)
  VAR_SetVariable(VAR_BAT_SOC,(int16_t)(mGmon.BatSoc),mBatValid);
  VAR_SetVariable(VAR_BAT_VOLTAGE_V10,(int16_t)(mGmon.BatVoltage*10),mBatValid);
  VAR_SetVariable(VAR_BAT_CURRENT_A10,(int16_t)(mGmon.BatCurrent*10),mBatValid);
  VAR_SetVariable(VAR_BAT_POWER_W,(int16_t)(mGmon.BatPower),mBatValid);
  VAR_SetVariable(VAR_BAT_ENERGY_WH,(int16_t)(mGmon.BatEnergy),mBatValid);

  VAR_SetVariable(VAR_SOLAR_POWER_W,(int16_t)(mGmon.SolarPower),mSolarValid);
  VAR_SetVariable(VAR_SOLAR_ENERGY_TODAY_10WH,(int16_t)(mGmon.SolarProd_Wh/10), mSolarValid);
  //VAR_SetVariable(VAR_CHARGING_A10,(int16_t)(10 * mGmon.SolarPower / mGmon.BatVoltage),1);  // !!! WARNING Used by TECHM ELHEATER

  VAR_SetVariable(VAR_LOAD_W,(int16_t)(mGmon.LoadPower),mLoadValid);
  VAR_SetVariable(VAR_CONS_TODAY_10WH,(int16_t)(mGmon.LoadCons_Wh/10), mLoadValid);
  VAR_SetVariable(VAR_LOAD_A100,mLoadA100,mLoadA100Valid);  // Auxiliary value just for WEB visualisation

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
