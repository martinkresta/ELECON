/*
 * ELECON.c
 *
 *  Created on: Nov 9, 2021
 *      Author: Martin
 */



#include "ELECON.h"
#include "VARS.h"
#include "SHUNT.h"



int16_t mSoc_pct100;
uint8_t mBattFullFlag;
int64_t mBattRemaining_mAs;  // miliamperseconds :-D
int32_t mTodayCons_Ws;    // wattseconds
uint8_t mChargingDisabledFlag;
uint8_t mSOCInitialisedFlag;
int32_t mBattEnergy_Wh;

void ELC_Init(void)
{
	mSoc_pct100 = 5000;
	mBattRemaining_mAs = BAT_EFF_CAPACITY_AH * AH2MAS / 2;
	mChargingDisabledFlag = 0;
	mSOCInitialisedFlag = 0;
	mTodayCons_Ws = 0;
}

void ELC_Update_1s(void)
{
	uint8_t invalid = 0;
	int16_t loadCurrennt_A10;
	// collect available inputs
	int16_t mpptCurrent_A10 = VAR_GetVariable(VAR_MPPT_BAT_CURRENT_A10, &invalid);
	int16_t shuntCurrent_A100 = VAR_GetVariable(VAR_SHUNT_CURRENT_A100, &invalid);
	int16_t railVoltage_V10 = VAR_GetVariable(VAR_MPPT_BAT_VOLTAGE_V100, &invalid)/10;
	//int16_t socBms1 = VAR_GetVariable(VAR_BMS1_SOC, &invalid);
	int16_t socBms1 = 75;
	int16_t socBms2 = VAR_GetVariable(VAR_BMS2_SOC, &invalid);




	if (invalid == 0)  // continue only if valid inputs
	{

		if (mSOCInitialisedFlag == 0 &&  socBms2 > 0)
		{
			// after SW restart init the SOC value with the value of BMS2 SOC
			mSOCInitialisedFlag = 1;
			mSoc_pct100 = socBms2*100;
			mBattRemaining_mAs = BAT_EFF_CAPACITY_AH * AH2MAS * mSoc_pct100 / 10000;
		}

		// Use the MPPT bat voltage as the battery voltage, and shunt curretn as battery current
		VAR_SetVariable(VAR_BAT_VOLTAGE_V10,railVoltage_V10,1);
		VAR_SetVariable(VAR_BAT_CURRENT_A10,shuntCurrent_A100 / 10,1);
		VAR_SetVariable(VAR_CHARGING_A10,mpptCurrent_A10,1);

		// Set the 100% SOC when one of the pack is full

		if (((socBms1 >= 99) || (socBms2 >= 99)) && mpptCurrent_A10 == 0 && mChargingDisabledFlag == 0)
		{
			mSoc_pct100 = 10000;
			mBattRemaining_mAs = BAT_EFF_CAPACITY_AH * AH2MAS;  // Convert Ah to mAs
			mChargingDisabledFlag = 1;
		}

		// when charging current is more than zero, reset charging disabled flag
		if (mpptCurrent_A10 != 0)
		{
			mChargingDisabledFlag = 0;
		}

		// calculate energy consumption
		mBattRemaining_mAs += SHUNT_GetIbat_mA();    // charged/consumed miliamperseconds during last second

		// calulate SOC
		mSoc_pct100 = mBattRemaining_mAs / (BAT_EFF_CAPACITY_AH * AH2MAS / 10000);
		mBattEnergy_Wh = (mBattRemaining_mAs * railVoltage_V10) / (AH2MAS * 10);

		VAR_SetVariable(VAR_BAT_SOC,mSoc_pct100/100,1);

		VAR_SetVariable(VAR_BAT_ENERGY_WH,mBattEnergy_Wh,1);



    // calculate load current and power
		loadCurrennt_A10 = ((mpptCurrent_A10 * 10) - shuntCurrent_A100)/10;
		VAR_SetVariable(VAR_LOAD_A10,loadCurrennt_A10,1);
		VAR_SetVariable(VAR_LOAD_W,(loadCurrennt_A10 * railVoltage_V10)/100,1);

		// cumulate daily consumption
		mTodayCons_Ws += (loadCurrennt_A10 * railVoltage_V10)/100;
		VAR_SetVariable(VAR_CONS_TODAY_WH,(int16_t)(mTodayCons_Ws/3600), 1);
	}

	else

	{
		// TBD invalidate some values
	}


}


void ELC_MidnightNow(void)
{
	// Reset counters at midnight
	mTodayCons_Ws = 0;

}
