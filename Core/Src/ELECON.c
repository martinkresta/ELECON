/*
 * ELECON.c
 *
 *  Created on: Nov 9, 2021
 *      Author: Martin
 */



#include "ELECON.h"
#include "VARS.h"
#include "SHUNT.h"
#include "BMS1.h"
#include "BMS2.h"
#include "COM.h"
#include "ADC.h"
#include "UI.h"
#include "RTC.h"



uint8_t mBattFullFlag;
int32_t mTodayCons_Ws;    // wattseconds
int32_t mTodaySolarYield_Ws;    // wattseconds

uint8_t mBatteryBalancedToday;
uint8_t mSOCInitialisedFlag;
uint8_t mChargingDisabledFlag;


// pack data

sPackInfo Pack1;  // baterie 5kWh
sPackInfo Pack2;  // baterie Doma
sStorageInfo Storage;    // celkova baterie




void ControlAuxBat(void);

void UpdatePack1Data(void);


void ELC_Init(void)
{
  Pack2.Soc_pct100 = 5000;
  Pack2.Available_mAs = PACK14KWH_EFF_CAPACITY_AH * AH2MAS / 2;
  Pack2.ChargingEnabled = 1;
  Pack2.DischargingEnabled = 1;

  Storage.Soc_pct100 = 5000;
  Storage.Available_mAs =  BAT_EFF_CAPACITY_AH * AH2MAS / 2;
}

void ELC_Update_1s(void)
{
	uint16_t invalid = 0;
	int16_t loadPowerW;
	int16_t SolarPowerW;
	// collect available inputs

	//int16_t socBms1 = VAR_GetVariable(VAR_BMS1_SOC, &invalid);
	int16_t socBms2 = VAR_GetVariable(VAR_BMS2_SOC, &invalid);
	int16_t optimalBalancingCurrent_A;  // optimal charging current during ongoing balancing
	int16_t optimalChargingCurrent_A;  // optimal charging current during the day, to reach 100% SOC at BAT_FULL_TARGET_HOUR
	int64_t remainingTime_s;
	sDateTime now;

	uint16_t bms1MaxVoltage_mV;
	uint16_t bms2MaxVoltage_mV;
	uint16_t maxCellVoltage_mV;
	uint8_t txdata[8];


	Pack2.Voltage_V10 = VAR_GetVariable(VAR_BMS2_VOLTAGE_V10, &invalid);
	Pack2.Current_mA = SHUNT_GetIbat_mA();
	Pack2.ChargingEnabled = BMS2_IsChargingEnabled();


	if (invalid == 0)  // continue only if valid inputs
	{

		if (mSOCInitialisedFlag == 0 &&  socBms2 > 0)
		{
			// after SW restart init the SOC value with the value of BMS2 SOC
			mSOCInitialisedFlag = 1;
			Pack2.Soc_pct100  = socBms2*100;
			Pack2.Available_mAs = PACK14KWH_EFF_CAPACITY_AH * AH2MAS * Pack2.Soc_pct100 / 10000;
		}

		// Set the 100% SOC when one of the pack is full
	//	if (((socBms1 >= 99 && socBms2 > 90) || (socBms2 >= 99)) && mpptCurrent_A10 == 0 && mChargingDisabledFlag == 0)
		if ((!Pack2.ChargingEnabled)  && mChargingDisabledFlag == 0)
		{
		  Pack2.Soc_pct100 = 10000;
		  Pack2.Available_mAs = PACK14KWH_EFF_CAPACITY_AH * AH2MAS;  // Convert Ah to mAs
			mChargingDisabledFlag = 1;
		}

		// when charging current is more than zero, reset charging disabled flag
		if (VAR_GetVariable(VAR_MPPT_BAT_CURRENT_A10, &invalid) != 0)
		{
			mChargingDisabledFlag = 0;
		}

		// calculate energy consumption
		Pack2.Available_mAs += SHUNT_GetIbat_mA();    // charged/consumed miliamperseconds during last second

		// limit to 100%
		if (Pack2.Available_mAs > (PACK14KWH_EFF_CAPACITY_AH * AH2MAS))
		{
		  Pack2.Available_mAs = PACK14KWH_EFF_CAPACITY_AH * AH2MAS;
		}

		// calculate PACK data

		Pack2.Soc_pct100 = Pack2.Available_mAs / (PACK14KWH_EFF_CAPACITY_AH * AH2MAS / 10000);
    Pack2.Energy_Wh = (Pack2.Available_mAs * Pack2.Voltage_V10) / (AH2MAS * 10);
    Pack2.Power_W = (Pack2.Voltage_V10 * Pack2.Current_mA) / 10000;

    VAR_SetVariable(VAR_BATPACK2_SOC,Pack2.Soc_pct100/100,1);
    VAR_SetVariable(VAR_BATPACK2_ENERGY_WH,Pack2.Energy_Wh,1);
    VAR_SetVariable(VAR_BATPACK2_CURRENT_A10,Pack2.Current_mA/100,1);
    VAR_SetVariable(VAR_BATPACK2_POWER_W, Pack2.Power_W,1);

		// calulate overall battery data (SOC etc)
    // average bat voltage
    Storage.Voltage_V10 = (Pack2.Voltage_V10 + VAR_GetVariable(VAR_BMS1_VOLTAGE_V10, &invalid))/2;
    // bat energy
    Storage.Energy_Wh = Pack2.Energy_Wh + VAR_GetVariable(VAR_BATPACK1_ENERGY_WH, &invalid);

    // bat current
    Storage.Current_mA = Pack2.Current_mA + VAR_GetVariable(VAR_SHUNT_PCK1_CURRENT_A100, &invalid) * 10;
    // bat power
    Storage.Power_W = (Storage.Voltage_V10 * Storage.Current_mA) / 10000;

    Storage.Available_mAs = Pack2.Available_mAs + ((VAR_GetVariable(VAR_BATPACK1_ENERGY_WH, &invalid) /  ( VAR_GetVariable(VAR_BMS1_VOLTAGE_V10, &invalid)/10)) * AH2MAS);
    // bat soc
    Storage.Soc_pct100 =  Storage.Available_mAs / (BAT_EFF_CAPACITY_AH * AH2MAS / 10000);



		VAR_SetVariable(VAR_BAT_SOC,Storage.Soc_pct100 /100,1);
		VAR_SetVariable(VAR_BAT_VOLTAGE_V10,Storage.Voltage_V10,1);
		VAR_SetVariable(VAR_BAT_CURRENT_A10,Storage.Current_mA/100,1);
		VAR_SetVariable(VAR_BAT_POWER_W,Storage.Power_W,1);
		VAR_SetVariable(VAR_BAT_ENERGY_WH,Storage.Energy_Wh,1);


		//  Collect solar data
		SolarPowerW = VAR_GetVariable(VAR_MPPT_SOLAR_POWER_W, &invalid) + VAR_GetVariable(VAR_AXPERT_SOLAR_W, &invalid);
		mTodaySolarYield_Ws += SolarPowerW;
		VAR_SetVariable(VAR_SOLAR_ENERGY_TODAY_10WH,(int16_t)(mTodaySolarYield_Ws/36000), 1);


    // calculate load current and power
		loadPowerW= SolarPowerW - Storage.Power_W;
		VAR_SetVariable(VAR_LOAD_W,loadPowerW,1);
    mTodayCons_Ws += loadPowerW;
		VAR_SetVariable(VAR_CONS_TODAY_10WH,(int16_t)(mTodayCons_Ws/36000), 1);








		// calculate optimal charging current during balancing
		if (mBatteryBalancedToday == 0)
		{
			optimalBalancingCurrent_A = 100;
			bms1MaxVoltage_mV = BMS1_GetMaxCellVoltage();
			bms2MaxVoltage_mV = BMS2_GetMaxCellVoltage();

			// Stage 1 : Support the cell top balancing by lowering charging current
			if (bms1MaxVoltage_mV > bms2MaxVoltage_mV)
			{
				maxCellVoltage_mV = bms1MaxVoltage_mV;
			}
			else
			{
				maxCellVoltage_mV = bms2MaxVoltage_mV;
			}
			if (maxCellVoltage_mV > CELL_BALANCE_MV)
			{
					optimalBalancingCurrent_A = 2 + ((CELL_MAX_MV - maxCellVoltage_mV) * 40/(CELL_MAX_MV - CELL_BALANCE_MV));
			}

			// set the 100% SOC if all cells exceed the target voltage
		//	if (BMS1_GetMinCellVoltage() >= CELL_TARGET_MV && BMS2_GetMinCellVoltage() >= CELL_TARGET_MV)
		  if (BMS2_GetMinCellVoltage() >= CELL_TARGET_MV)
			{
				mBatteryBalancedToday = 1;
				Pack2.Soc_pct100 = 10000;
				Pack2.Available_mAs = PACK14KWH_EFF_CAPACITY_AH * AH2MAS;  // Convert Ah to mAs
				//Storage.Available_mAs = BAT_EFF_CAPACITY_AH * AH2MAS;  // Convert Ah to mAs
			}

			// stage 2: Maximal utilization of available PV energy:

			// calculate optimal charging current to reach full SOC at certain time

			// calculate remaining time in seconds

			now = RTC_GetTime();
			if (now.Hour < BAT_FULL_TARGET_HOUR && now.Month > 2 && now.Month < 11)  // Only if it is not a DARK_SEASON from November to February
			{
				remainingTime_s = (BAT_FULL_TARGET_HOUR - now.Hour - 1) * 3600 + (60 - now.Minute)*60;
				optimalChargingCurrent_A = ((BAT_EFF_CAPACITY_AH * AH2MAS) - Storage.Available_mAs) / (remainingTime_s * 1000);

				optimalChargingCurrent_A -= 4;  // pre-compensate quantization error  (10A) by ELHEATER
			}
			else
			{
				//optimalChargingCurrent_A = 100;
			}

		}
		else
		{
			optimalBalancingCurrent_A = 0;
			optimalChargingCurrent_A = 0;
		}


		// electric heater load control  (to lower charging current)

		// Send status of balancedtoday and optimal balancing current to TECHM
		txdata[0] = 0;
		txdata[1] = mBatteryBalancedToday;
		if (optimalChargingCurrent_A >= optimalBalancingCurrent_A)
		{
			txdata[2] = optimalBalancingCurrent_A >> 8;
			txdata[3] = optimalBalancingCurrent_A & 0xFF;
		}
		else
		{
			txdata[2] = optimalChargingCurrent_A >> 8;
			txdata[3] = optimalChargingCurrent_A & 0xFF;
		}
		txdata[4] = 0;
		txdata[5] = 0;
		txdata[6] = 0;
		txdata[7] = 0;
		COM_SendMessage(CMD_BALANCE_INFO, txdata, 8);

		// control EV charging
	}

	else

	{
		// TBD invalidate some values
	}

	ControlAuxBat();


}


void ELC_MidnightNow(void)
{
	// Reset counters at midnight
	mTodayCons_Ws = 0;
	mTodaySolarYield_Ws = 0;
	mBatteryBalancedToday = 0;
}


void UpdatePack1Data(void)
{

}

void ControlAuxBat(void)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint16_t VbatRaw;
	//uint16_t IbatRaw;
	uint8_t BackupOn = 0;
	BackupOn = HAL_GPIO_ReadPin(BCKP_STATE_GPIO_Port, BCKP_STATE_Pin);
	// Read Aux battery voltage

	VbatRaw = ADC_GetValue(ADC_CHANNEL_AUX_BAT_V);  // raw ADC result
	double Vbat_mV = (ADC_VREF_MV/4096.0 * VbatRaw * 12) / 2.44 ;  // convert to milivolts
	// Read Aux battery current
	//IbatRaw = ADC_GetValue(ADC_CHANNEL_AUX_BAT_I);  // raw ADC result
	//double Ibat_mA = (ADC_VREF_MV/4096.0 * IbatRaw) * 2.128 ;  // convert to miliamperes

	// check status of backup
	if (BackupOn == 1)
	{
		UI_LED_B_SetMode(eUI_BLINKING_FAST);   // signalization of main battery powerdown
	}
	else
	{
		UI_LED_B_SetMode(eUI_OFF);
	}

	// enable/disable charging
	if(Vbat_mV > 13900)
	{
		HAL_GPIO_WritePin(CHARGE_ENA_GPIO_Port, CHARGE_ENA_Pin, GPIO_PIN_RESET);  // disable charging
	}
	else if (Vbat_mV < 13000)
	{
		HAL_GPIO_WritePin(CHARGE_ENA_GPIO_Port, CHARGE_ENA_Pin, GPIO_PIN_SET);  // enable charging
	}

	// enable disable backup
	if(Vbat_mV > 12000)
	{
		GPIO_InitStruct.Pin = BCKP_ENA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		HAL_GPIO_WritePin(BCKP_ENA_GPIO_Port, BCKP_ENA_Pin, GPIO_PIN_RESET);  // enable backup
	}
	else if (Vbat_mV < 11000)
	{
		 /*Configure GPIO pins : BCKP_ENA_Pin */
		GPIO_InitStruct.Pin = BCKP_ENA_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       // HiZ = disable backup
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}
}

