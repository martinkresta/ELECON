/*
 *  MPPT.c
 *
 *  Created on: Oct 30, 2021
 *      Author: Martin
 *       Brief: Module for communication with Victron SmartSolar MPPT
 */

#include "MPPT.h"
#include "VARS.h"

static uint8_t mRxData[DMA_LENGTH];  // buffer for reception of raw BMS data
static UART_HandleTypeDef* mMpptUart;
static uint8_t mTxData[DMA_LENGTH];

static uint16_t mScanRegisters[10];
static uint8_t mNumOfScannedRegisters;
static uint8_t mScanIndex;
static uint8_t mTxBusy;
static uint8_t mNewDataReady;
static uint8_t mRecLength;



// private methods
static void DecodeMessage(void);
uint32_t Hex2Uint(uint8_t* string, uint8_t length);
void Uint2Hex(uint32_t value, uint8_t* string, uint8_t length);
void SendMessage(sTxMsg* msg);
uint8_t validate16(uint16_t val, uint8_t* checksum, uint8_t* sum);
uint8_t validate32(uint32_t val, uint8_t* checksum, uint8_t* sum);


// Initialization of the MPPT module
void MPPT_Init(UART_HandleTypeDef* huart)
{
	HAL_StatusTypeDef UartRetval;
	uint32_t UartError;
	mNewDataReady = 0;
	mRecLength = 0;
	mScanIndex = 0;
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


	// test message
/*	mRxData[0] = ':';
	mRxData[1] = '7';
	mRxData[2] = 'F';
	mRxData[3] = '0';
	mRxData[4] = 'E';
	mRxData[5] = 'D';
	mRxData[6] = '0';
	mRxData[7] = '0';
	mRxData[8] = '9';
	mRxData[9] = '6';
	mRxData[10] = '0';
	mRxData[11] = '0';
	mRxData[12] = 'D';
	mRxData[13] = 'B';
	mRxData[14] = '\n';*/

	UartRetval = HAL_UARTEx_ReceiveToIdle_DMA(mMpptUart, mRxData, DMA_LENGTH);
	if(UartRetval  != HAL_OK)
	{
		UartError = HAL_UART_GetError(mMpptUart);
	}

}

// Update function, to the called periodically by the scheduler
void MPPT_Update_100ms(void)
{
	HAL_StatusTypeDef UartRetval;
	uint32_t UartError;
	sTxMsg txMsg;
	txMsg.cmd = MPPT_CMD_GET;
	static uint8_t validflag = 0;

	if (mNewDataReady)
	{
		if (mRecLength >= 1)
		{
			DecodeMessage();
		}
		mNewDataReady = 0;
	}

	UartRetval = HAL_UARTEx_ReceiveToIdle_DMA(mMpptUart, mRxData, DMA_LENGTH);
	if(UartRetval != HAL_OK)
	{
		UartError = HAL_UART_GetError(mMpptUart);
	}

	// Send variable GET command



	txMsg.reg = mScanRegisters[mScanIndex++];
	if (mScanIndex >= mNumOfScannedRegisters)
	{
		mScanIndex = 0;
	}
	SendMessage(&txMsg);
}



void DecodeMessage(void)
{
	uint16_t reg;
	uint32_t value;
	uint8_t checksum = 0;
	uint8_t flags;
	if(mRxData[0] == ':')  // check that message is HEX protocol
	{
		uint8_t cmd = Hex2Uint(&(mRxData[1]), 1);
		checksum += cmd;
		switch (cmd)
		{
			case MPPT_CMD_GET:
				reg = Hex2Uint(&(mRxData[2]), 4);
				checksum += reg & 0xFF;
				checksum += (reg & 0xFF00) >> 8;
				flags = Hex2Uint(&(mRxData[6]), 2);
				checksum += flags & 0xFF;
				DecodeRegisterValue(reg, &(mRxData[8]), &checksum);
				break;
			default: // other commands not supported yet
				break;
		}

	}
	else  // text protocol
	{
		// text messages are ignored
	}

}

void DecodeRegisterValue(uint16_t reg , uint8_t* value, uint8_t* sum)
{
	uint16_t u16;
	int16_t s16;
	uint32_t u32;
	eValType type;
	switch (reg)
	{
		case MPPT_REG_MAX_CHARGING_CURRENT:   // u16,  0.01 Amps
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_MAX_BAT_CURRENT_A10, u16, 1);
			}
			break;
		case MPPT_REG_TEMP:     //  s16    0.01 C
			// TBD
			break;
		case MPPT_REG_CHARGER_CURRENT:  //  u16  0.1 A
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_BAT_CURRENT_A10, u16, 1);
			}
			break;
		case MPPT_REG_CHARGER_VOLTAGE:  //  u16   0.01 V
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_BAT_VOLTAGE_V100, u16, 1);
			}
			break;
		case MPPT_REG_YIELD_TODAY:    // u16  10 Wh
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_YIELD_TODAY_10WH, u16, 1);
			}
			break;
		case MPPT_REG_MAX_POWER_TODAY: // u16  W
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_MAX_TODAY_W, u16, 1);
			}
			break;
		case MPPT_REG_SOLAR_POWER	:  //  u32 0.01 W
			u32 = Hex2Uint(value, 8);
			if (validate16(u32,value+8,sum))
			{
				VAR_SetVariable(VAR_MPPT_SOLAR_POWER_W, u32/100, 1);
			}
			break;
		case MPPT_REG_SOLAR_VOLTAGE: // u16 0.01 V
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_SOLAR_VOLTAGE_V100, u16, 1);
			}
			break;
		case MPPT_REG_SOLAR_CURRENT:  // u16  0.1  A
			u16 = Hex2Uint(value, 4);
			if (validate16(u16,value+4,sum))
			{
				VAR_SetVariable(VAR_MPPT_SOLAR_CURRENT_A10, u16, 1);
			}
			break;
		case MPPT_REG_SOLAR_MAX_VOLTAGE:  // u16 0.01 V
			// TBD
			break;
		default:
			 // not supported registers
			break;
	}
}


uint8_t validate16(uint16_t val, uint8_t* checksum, uint8_t* sum)
{
	*sum += val & 0xFF;
	*sum += (val & 0xFF00) >> 8;
	*sum += Hex2Uint(checksum, 2);
	if (*(checksum+2) == '\n'  && *sum == MPPT_CHECKSUM_RES)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t validate32(uint32_t val, uint8_t* checksum, uint8_t* sum)
{
	*sum += val && 0xFF;
	*sum += (val && 0xFF00) >> 8;
	*sum += (val && 0xFF0000) >> 16;
	*sum += (val && 0xFF000000) >> 24;
	*sum += *checksum;
	if (*(checksum+1) == '\n'  && *sum == MPPT_CHECKSUM_RES)
	{
		return 1;
	}
	else
	{
		return 0;
	}
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


void SendMessage(sTxMsg* msg)
{
	uint8_t checksum = MPPT_CHECKSUM_RES;
	uint8_t txLength = 0;
	uint8_t i = 0;
	if (mTxBusy) return;
	mTxData[i++] = ':';
	Uint2Hex((uint32_t)msg->cmd, &(mTxData[i]), 1);
	checksum -= msg->cmd;
	i += 1;
	if (msg->cmd == MPPT_CMD_GET || msg->cmd == MPPT_CMD_SET)
	{
		Uint2Hex((uint32_t)msg->reg, &(mTxData[i]), 4);
		checksum -= msg->reg & 0xFF;
		checksum -= ((msg->reg & 0xFF00) >> 8);
		i += 4;

		Uint2Hex(0, &(mTxData[i]), 2);  // flags  0x00
		i += 2;

		if (msg->cmd == MPPT_CMD_SET)
		{
			switch (msg->datatype)
			{
				case etu8:
					Uint2Hex(msg->value, &(mTxData[i]), 2);
					checksum -= msg->value & 0xFF;
					i += 2;
					break;
				case etu16:
					Uint2Hex(msg->value, &(mTxData[i]), 4);
					checksum -= msg->value & 0xFF;
					checksum -= ((msg->value & 0xFF00) >> 8);
					i += 4;
					break;
				case etu32:
					Uint2Hex(msg->value, &(mTxData[i]), 8);
					checksum -= msg->value & 0xFF;
					checksum -= ((msg->value & 0xFF00) >> 8);
					checksum -= ((msg->value & 0xFF0000) >> 16);
					checksum -= ((msg->value & 0xFF000000) >> 24);
					i += 8;
					break;
				case ets16:
					//TBD
					break;
				case ets32:
					// TBD
					break;
			}
		}

	}

	Uint2Hex((uint32_t)checksum, &(mTxData[i]), 2);  // append checksum
	i += 2;
	mTxData[i] = '\n';  // append end character
	txLength = i+1;

	HAL_UART_Transmit_DMA(mMpptUart, &mTxData, txLength);

}


// Converts hex string of <length> characters to an integer value
uint32_t Hex2Uint(uint8_t* string, uint8_t length)
{
		uint32_t res = 0;
		uint8_t i = 0;
		uint8_t *c = string;

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
	uint8_t nibble = 0, byte = 0;
	uint8_t c,i;
	uint32_t val = 0xFFFFFFFF;

	val <<= length*4;
	val = ~val ;
	val &= value;


	if (length == 1)
	{
		string[0] = (val) <= 9 ? val + '0' : val - 10 + 'A';
	}
	for (i = 0; i < length; i++)
	{
		byte += (val & (0x0F << (i*4))) >> ((i/2)*8);
		if (i % 2 == 1)
		{
			nibble = (byte & 0xF0) >> 4;  // high nibble
			string[i-1] = (nibble) <= 9 ? nibble + '0' : nibble - 10 + 'A';
			nibble = (byte & 0xF);  // low nibble
			string[i] = (nibble) <= 9 ? nibble + '0' : nibble - 10 + 'A';
			byte = 0;
		}
	}
}
