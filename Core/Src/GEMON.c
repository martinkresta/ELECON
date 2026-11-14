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
#include "VARS2.h"


sGemon mGmon;
sElUtil mEutil;

// Validity of the published output groups (1 = last computation used only valid inputs).
// While a group is invalid, mGmon keeps its last valid values and they are published with valid = 0.
static uint8_t mBatValid;
static uint8_t mSolarValid;
static uint8_t mLoadValid;
static float   mTotalCapacity;   // last valid total capacity [Wh] (used by the optimal charging calculation)
static float   mLoadCurrent_A;   // load current, computed only while load and battery are valid (last value kept)

static sVarsGlobalEnergy* mpOut;            /* produced: Battery, Solar, Load */
static const sVarsLocalEnergy* mpHome;      /* read: HomeEnergy (own LEMON) */
static const sVarsLocalEnergy* mpWorkshop;  /* read: WorkshopEnergy (ELECON_D) */

static void SendBalanceInfo(void);

void GEMON_Init(sVarsGlobalEnergy* pOut, const sVarsLocalEnergy* pHome, const sVarsLocalEnergy* pWorkshop)
{
  mpOut = pOut;
  mpHome = pHome;
  mpWorkshop = pWorkshop;
  mEutil.ChargeTargetHour = 15;
}

void GEMON_Update_1s(void)
{
  sDateTime now;
  const sVarsLocalEnergyStorage* h = &mpHome->Storage;
  const sVarsLocalEnergyStorage* w = &mpWorkshop->Storage;
  uint8_t batInputsValid, loadInputsValid, solarInputsValid;
  float totalCapacity;
  float optChargePower;
  float totalOptBalanceCurrent;
  float batEnergy, batVoltage, batPower, batCurrent;
  float loadPower, loadCons_Wh;
  float solarPower, solarProd_Wh;

  // calculate

  // battery group
  batInputsValid = h->Energy_Wh.valid && w->Energy_Wh.valid && h->Voltage_V.valid && w->Voltage_V.valid &&
               h->Capacity_Ah.valid && w->Capacity_Ah.valid && h->Power_W.valid && w->Power_W.valid &&
               h->Current_A.valid && w->Current_A.valid;
  batEnergy = h->Energy_Wh.value + w->Energy_Wh.value;
  batVoltage = (h->Voltage_V.value + w->Voltage_V.value) / 2;
  totalCapacity = (h->Capacity_Ah.value + w->Capacity_Ah.value) * batVoltage;
  batPower = h->Power_W.value + w->Power_W.value;
  batCurrent = h->Current_A.value + w->Current_A.value;
  if (batInputsValid && totalCapacity > 0 && batVoltage > 0)
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
  loadInputsValid = mpHome->Load.Power_W.valid && mpWorkshop->Load.Power_W.valid &&
                mpHome->Load.Cons_Wh.valid && mpWorkshop->Load.Cons_Wh.valid;
  loadPower = mpHome->Load.Power_W.value + mpWorkshop->Load.Power_W.value;
  loadCons_Wh = mpHome->Load.Cons_Wh.value + mpWorkshop->Load.Cons_Wh.value;
  if (loadInputsValid)
  {
    mGmon.LoadPower = loadPower;
    mGmon.LoadCons_Wh = loadCons_Wh;
    mLoadValid = 1;
  }
  else
  {
    mLoadValid = 0;
  }
  if (mLoadValid && mBatValid)   // load current also needs the battery voltage (> 0 guaranteed by mBatValid)
  {
    mLoadCurrent_A = mGmon.LoadPower / mGmon.BatVoltage;
  }
  // else: last value kept, published with the load validity (decision 3, direct access)

  // solar group
  solarInputsValid = mpHome->Pv.Power_W.valid && mpWorkshop->Pv.Power_W.valid &&
                 mpHome->Pv.Prod_Wh.valid && mpWorkshop->Pv.Prod_Wh.valid;
  solarPower = mpHome->Pv.Power_W.value + mpWorkshop->Pv.Power_W.value;
  solarProd_Wh = mpHome->Pv.Prod_Wh.value + mpWorkshop->Pv.Prod_Wh.value;
  if (solarInputsValid)
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
  totalOptBalanceCurrent = h->OptCharging_A.value + w->OptCharging_A.value;  // calculated by lemon to support balancing
  mEutil.Strg1.BalancedToday = (uint8_t)h->BalancedToday.value;
  mEutil.Strg2.BalancedToday = (uint8_t)w->BalancedToday.value;


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
  mpOut->Battery.Soc_pct.value = mGmon.BatSoc;
  mpOut->Battery.Voltage_V.value = mGmon.BatVoltage;
  mpOut->Battery.Current_A.value = mGmon.BatCurrent;
  mpOut->Battery.Power_W.value = mGmon.BatPower;
  mpOut->Battery.Energy_Wh.value = mGmon.BatEnergy;
  VARS2_SetGroupValid(&mpOut->Battery, mBatValid);

  mpOut->Solar.Power_W.value = mGmon.SolarPower;
  mpOut->Solar.EnergyToday_Wh.value = mGmon.SolarProd_Wh;
  VARS2_SetGroupValid(&mpOut->Solar, mSolarValid);

  mpOut->Load.Power_W.value = mGmon.LoadPower;
  mpOut->Load.ConsToday_Wh.value = mGmon.LoadCons_Wh;
  mpOut->Load.Current_A.value = mLoadCurrent_A;
  VARS2_SetGroupValid(&mpOut->Load, mLoadValid);
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
