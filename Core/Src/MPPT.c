/*
 *  MPPT.c
 *
 *  Created on: Oct 30, 2021
 *      Author: Martin
 *       Brief: Module for communication with Victron SmartSolar MPPT
 */

#include "MPPT.h"
#include "VARS.h"

static uint8_t mRxData[DMA_REC_LENGTH +2];  // buffer for reception of raw BMS data
static UART_HandleTypeDef* mMpptUart;
static uint8_t mTxData[20];

static uint16_t mScanRegisters[10];
static uint8_t mNumOfScannedRegisters;
static uint8_t mScanIndex;
static uint8_t mTxBusy;



// private methods
static void DecodeData(void);
static uint8_t IsChecksumValid(void);
uint32_t Hex2Uint(uint8_t* string, uint8_t length);
void Uint2Hex(uint32_t value, uint8_t* string, uint8_t length);
void SendMessage(sRxMsg msg);


// Initialization of the MPPT module
void MPPT_Init(UART_HandleTypeDef* huart)
{
	HAL_StatusTypeDef UartRetval;
	uint32_t UartError;
	mNewDataReady = 0;
	mRecLength = 0;
	mMpptUart = huart;

	mNumOfScannedRegisters = 0;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_CHARGER_CURRENT;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_CHARGER_VOLTAGE;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_YIELD_TODAY;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_MAX_POWER_TODAY;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_SOLAR_POWER;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_SOLAR_VOLTAGE;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_SOLAR_CURRENT;
	mScanRegisters[mNumOfScannedRegisters++] = MPPT_REG_SOLAR_MAX_VOLTAGE;



	UartRetval = HAL_UARTEx_ReceiveToIdle_DMA(mMpptUart, mRxData, DMA_REC_LENGTH);
	if(UartRetval  != HAL_OK)
	{
		UartError = HAL_UART_GetError(mMpptUart);
	}

}

// Update function, to the called periodically by the scheduler
void MPPT_Update_500ms(void)
{
	HAL_StatusTypeDef UartRetval;
	uint32_t UartError;
	static uint8_t validflag = 0;

	if (mNewDataReady)
	{
		if (mRecLength >= 1)
		{
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
		mNewDataReady = 0;
	}
	UartRetval = HAL_UARTEx_ReceiveToIdle_DMA(mMpptUart, mRxData, DMA_REC_LENGTH);
	if(UartRetval  != HAL_OK)
	{
		UartError = HAL_UART_GetError(mMpptUart);
	}

	//  TBD VAR_SetVariable(VAR_BMS1_VOLTAGE_V10, mLiveData.VoltageTotal_mV/100, validflag);


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

	// TBD
}



void MPPT_UartRxCallback(uint16_t reclength)
{
	mNewDataReady = 1;
	mRecLength = reclength;
}

void MPPT_UartTxCallback(void)
{
	mTxBusy = 0;
}


void SendMessage(sTxMsg msg)
{
	uint8_t checksum = 0x55;
	uint8_t txLength = 0;
	uint8_t i = 0;
	if (mTxBusy) return;
	mTxData[i++] = ':';
	Uint2Hex((uint32_t)msg.cmd, &(mTxData[i]), 1);
	checksum -= msg.cmd;
	i += 1;
	if (msg.cmd == MPPT_CMD_GET || msg.cmd == MPPT_CMD_SET)
	{
		Uint2Hex((uint32_t)msg.reg, &(mTxData[i]), 4);
		checksum -= msg.reg & 0xFF;
		checksum -= ((msg.reg & 0xFF00) >> 8);
		i += 4;

		Uint2Hex(0, &(mTxData[i]), 2);  // flags  0x00
		i += 2;

		if (msg.cmd == MPPT_CMD_SET)
		{
			switch (msg.datatype)
			{
				case u8:
					Uint2Hex(msg.value, &(mTxData[i]), 2);
					checksum -= msg.value & 0xFF;
					i += 2;
					break;
				case u16:
					Uint2Hex(msg.value, &(mTxData[i]), 4);
					checksum -= msg.value & 0xFF;
					checksum -= ((msg.value & 0xFF00) >> 8);
					i += 4;
					break;
				case u32:
					Uint2Hex(msg.value, &(mTxData[i]), 8);
					checksum -= msg.value & 0xFF;
					checksum -= ((msg.value & 0xFF00) >> 8);
					checksum -= ((msg.value & 0xFF0000) >> 16);
					checksum -= ((msg.value & 0xFF000000) >> 24);
					i += 8;
					break;
				case s16:
					//TBD
					break;
				case s32:
					// TBD
					break;
			}
		}


	}

	Uint2Hex((uint32_t)checksum, &(mTxData[i]), 2);  // append checksum
	i += 2;
	mTxData[i] = '\n';  // append end character
	txLength = i+1;

}


// Converts hex string of <length> characters to an integer value
uint32_t Hex2Uint(uint8_t* string, uint8_t length)
{
		uint32_t res = 0;
		uint8_t i = 0;
		//uint8_t *c = string;

		uint8_t byte;
		if (length == 1)
		{
			res =  (*c <= '9') ? *c - '0': (*c - 'A') + 10;
		}
		else
		{
			for (i = 0; i < length ; i++)
			{
				byte <<= 4;
				byte +=  (*c <= '9') ? *c - '0': (*c - 'A') + 10;
				if (i % 2 == 1)  // converting from little endian
				{
					res += byte << ((i-1)*4);
				}
				c++;
			}
		}
		return res;
}


// Converts an integer value to HEX string containing <length> characters  (Little Endian)
void Uint2Hex(uint32_t value, uint8_t* string, uint8_t length)
{
	uint8_t nibble, byte;
	uint8_t c,i;
	uint8_t val = 0xFFFFFFFF;

	val <<= length*4;
	val = ~val ;


	if (length = 1)
	{
		string[0] = (val) <= 9 ? val + '0' : val - 10 + 'A';
	}
	for (i = 0; i < length; i++)
	{
		byte << 4;
		byte += (val & (0x0F << (i*4))) >> i*4;
		if (i % 2 == 1)
		{
			nibble = (byte & 0xF0) >> 4;  // high nibble
			string[i-1] = (nibble) <= 9 ? nibble + '0' : nibble - 10 + 'A';
			nibble = (byte & 0xF);  // low nibble
			string[i] = (nibble) <= 9 ? nibble + '0' : nibble - 10 + 'A';
		}
	}

\\
}
