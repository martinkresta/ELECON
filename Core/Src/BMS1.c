/*
 * BMS1.h
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 *       Brief: Module for one-way communication with BMS on battery pack ()
 */

#include "BMS1.h"
#include "VARS.h"

static uint8_t mRxData[DMA_REC_LENGTH +2];  // buffer for reception of raw BMS data
static uint8_t* mBmsData;
static uint8_t mNewDataReady;
static uint8_t mRecLength;
static uint8_t mPackInfoValid;

static sPackInfo mPackInfo;
static sLiveData mLiveData;
static sCell Cells[16];
static UART_HandleTypeDef* mBmsUart;


static void DecodeData(void);
static uint8_t IsChecksumValid(void);


// Initialization of the BMS monitoring module
void BMS1_Init(UART_HandleTypeDef* huart)
{
	HAL_StatusTypeDef UartRetval;
	uint32_t UartError;
	mPackInfoValid = 1;
	mNewDataReady = 0;
	mRecLength = 0;
	mBmsUart = huart;


	UartRetval = HAL_UARTEx_ReceiveToIdle_DMA(mBmsUart, mRxData, DMA_REC_LENGTH);
	if(UartRetval  != HAL_OK)
	{
		UartError = HAL_UART_GetError(mBmsUart);
	}

}

// Update function, to the called periodically by the scheduler
void BMS1_Update_500ms(void)
{
	HAL_StatusTypeDef UartRetval;
	uint32_t UartError;
	static uint8_t validflag = 0;

	if (mNewDataReady)
	{
		if (mRecLength >= (BMS_DATA_LENGTH - 1))
		{
			if (mRecLength == 58)
			{
				mBmsData = &(mRxData[0]);
			}
			else if (mRecLength == 59)
			{
				mBmsData = &(mRxData[1]);
			}

			if (1 == IsChecksumValid())
			{
				DecodeData();
				validflag = 1;
			}
			else
			{
				// TBD, report invalid checksum
				validflag = 0;
			}
		}
		else // incomplete message - ignore it
		{
			validflag = 0;
			// TBD, report the thing
		}

		mNewDataReady = 0;
	}
	UartRetval = HAL_UARTEx_ReceiveToIdle_DMA(mBmsUart, mRxData, DMA_REC_LENGTH);
	if(UartRetval  != HAL_OK)
	{
		UartError = HAL_UART_GetError(mBmsUart);
	}

	VAR_SetVariable(VAR_BMS1_VOLTAGE_V10, mLiveData.VoltageTotal_mV/100, validflag);
	VAR_SetVariable(VAR_BMS1_SOC, mLiveData.SOC, validflag);
	VAR_SetVariable(VAR_BMS1_CURRENT_A10, mLiveData.BatteryCurrent_mA/100, validflag);
	VAR_SetVariable(VAR_BMS1_ENERGY_STORED_WH, mLiveData.Energystored_Wh, validflag);
	VAR_SetVariable(VAR_BMS1_TODAY_ENERGY_WH, mLiveData.TodayCharging_Wh, validflag);


	VAR_SetVariable(VAR_BMS1_CELL1_MV, Cells[0].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL2_MV, Cells[1].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL3_MV, Cells[2].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL4_MV, Cells[3].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL5_MV, Cells[4].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL6_MV, Cells[5].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL7_MV, Cells[6].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL8_MV, Cells[7].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL9_MV, Cells[8].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL10_MV, Cells[9].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL11_MV, Cells[10].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL12_MV, Cells[11].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL13_MV, Cells[12].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL14_MV, Cells[13].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL15_MV, Cells[14].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL16_MV, Cells[15].Voltage_mV, validflag);
	VAR_SetVariable(VAR_BMS1_CELL1_C, Cells[0].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL2_C, Cells[1].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL3_C, Cells[2].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL4_C, Cells[3].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL5_C, Cells[4].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL6_C, Cells[5].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL7_C, Cells[6].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL8_C, Cells[7].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL9_C, Cells[8].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL10_C, Cells[9].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL11_C, Cells[10].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL12_C, Cells[11].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL13_C, Cells[12].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL14_C, Cells[13].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL15_C, Cells[14].Temp_C, validflag);
	VAR_SetVariable(VAR_BMS1_CELL16_C, Cells[15].Temp_C, validflag);

}


// checks the data in receive buffer, returns 1 if chksm is valid, 0 otherwise
uint8_t IsChecksumValid(void)
{
	uint8_t i, sum;
	sum = 0;
	for (i = 0; i < (BMS_DATA_LENGTH - 1); i++)
	{
		sum += mBmsData[i];
	}
	if (mBmsData[BMS_DATA_LENGTH - 1] == sum)
	{
		return 1;  // checksum valid
	}
	else
	{
		return 0;  // checksum invalid
	}

}

void DecodeData(void)
{
//	char message[50];

	if (mPackInfoValid == 1)  // we can store Live data
	{

		// readable version

		mLiveData.VoltageTotal_mV = 5 * ((mBmsData[0]<<16)|(mBmsData[1]<<8)|(mBmsData[2]));
	//	mLiveData.ChargingCurrent_mA = (((mBmsData[4]<<8) | mBmsData[5])) * 125;  // TBD, add sign
	//	mLiveData.DischargingCurrent_mA = (((mBmsData[7]<<8) | mBmsData[8])) * 125;  // TBD, add sign
		if (mBmsData[4] != 'X')
		{
			mLiveData.ChargingCurrent_mA = (((mBmsData[4]<<8) | mBmsData[5])) * 125;
			if(mBmsData[4] == '-') mLiveData.ChargingCurrent_mA *= -1;
		}
		if (mBmsData[6] != 'X')
		{
			mLiveData.DischargingCurrent_mA = (((mBmsData[7]<<8) | mBmsData[8])) * 125;
			if(mBmsData[6] == '-') mLiveData.DischargingCurrent_mA *= -1;
		}
		if (mBmsData[9] != 'X')
		{
			mLiveData.BatteryCurrent_mA = (((mBmsData[10]<<8) | mBmsData[11])) * 125;
			if(mBmsData[9] == '-') mLiveData.BatteryCurrent_mA *= -1;
		}
		mLiveData.SOC = mBmsData[40];
		mLiveData.TodayCharging_Wh = (mBmsData[31]<<16)|(mBmsData[32]<<8)|(mBmsData[33]);
		mLiveData.TodayDischarging_Wh = (mBmsData[37]<<16)|(mBmsData[38]<<8)|(mBmsData[39]);
		mLiveData.Energystored_Wh = (mBmsData[34]<<16)|(mBmsData[35]<<8)|(mBmsData[36]);
		mLiveData.Status = mBmsData[30];
		mLiveData.TotalCharging_kWh = (mBmsData[41]<<16)|(mBmsData[42]<<8)|(mBmsData[43]);
		mLiveData.TotalDischarging_kWh = (mBmsData[44]<<16)|(mBmsData[45]<<8)|(mBmsData[46]);
		Cells[(mBmsData[24] - 1)].Voltage_mV = 5 * ((mBmsData[26]<<8) | mBmsData[27]);
		Cells[(mBmsData[24] - 1)].Temp_C= ((mBmsData[28]<<8) | mBmsData[29]) - 0x0114;
	}
	else
	{
		mPackInfo.NumOfCells =  mBmsData[25];
		mPackInfo.Capacity_Wh = 100 * ((mBmsData[49]<<8) | mBmsData[50]);
		mPackInfo.Vmin_mV = 5 * ((mBmsData[51]<<8) | mBmsData[52]);
		mPackInfo.Vmax_mV = 5 * ((mBmsData[53]<<8) | mBmsData[54]);
		mPackInfo.Vbalance_mV = 5 * ((mBmsData[55]<<8) | mBmsData[56]);
	//	Cells = malloc(mPackInfo.NumOfCells * sizeof(sCell));
		if (Cells != NULL)
		{
			mPackInfoValid = 1;
		}
		else
		{
			// TBD: Error: out of memory
		}

	}
}



void BMS1_UartRxCallback(uint16_t reclength)
{
	mNewDataReady = 1;
	mRecLength = reclength;
}
