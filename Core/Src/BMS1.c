/*
 * BMS1.h
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 *       Brief: Module for one-way communication with BMS on battery pack ()
 */

#include "BMS1.h"

uint8_t mRxData[DMA_REC_LENGTH];  // buffer for reception of raw BMS data
uint8_t* mBmsData;
uint8_t mNewDataReady;
uint8_t mRecLength;
volatile uint8_t mPackInfoValid;

sPackInfo mPackInfo;
sLiveData mLiveData;
sCell Cells[16];
UART_HandleTypeDef* mBmsUart;


void DecodeData(void);
uint8_t IsChecksumValid(void);


// Initialization of the BMS monitoring module
void BMS1_Init(UART_HandleTypeDef* huart)
{
	mPackInfoValid = 1;
	mNewDataReady = 0;
	mRecLength = 0;
	mBmsUart = huart;



	HAL_UART_Receive_DMA(mBmsUart, mRxData, DMA_REC_LENGTH);


/*	// enable receiver end receive interrupts
	BMS_UART->CR1 &= ~0x1; //  UE  = 0
	BMS_UART->CR3 |= 0x1000; // OVRDIS - disable overrun detection
	BMS_UART->CR1 = 0x15; //  IDLEIE, RE, UE*/

}

// Update function, to the called periodically by the scheduler
void BMS1_Update_500ms(void)
{
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
			}
			else
			{
				// TBD, report invalid checksum
			}
		}
		else // incomplete message - ignore it
		{
			// TBD, report the thing
		}


/*		BMS_UART->RQR |= 0x08; //   RXFRQ    (receive data flush)
		BMS_UART->ICR = 0x10;  // clear the IDLE flag
	//	BMS_UART->CR1 = 0x15; //  IDLEIE, RE, UE
		// Re-enable DMA receiver
		DMA1_Channel3->CNDTR = DMA_REC_LENGTH;
		DMA1_Channel3->CCR |= DMA_CCR_EN;
*/
		HAL_UART_Receive_DMA(mBmsUart, mRxData, DMA_REC_LENGTH);

	  mNewDataReady = 0;
	}

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
		mLiveData.ChargingCurrent_mA = (((mBmsData[4]<<8) | mBmsData[5])) * 125;  // TBD, add sign
		mLiveData.DischargingCurrent_mA = (((mBmsData[7]<<8) | mBmsData[8])) * 125;  // TBD, add sign
		mLiveData.SOC = mBmsData[40];
		mLiveData.TodayCharging_Wh = (mBmsData[31]<<16)|(mBmsData[32]<<8)|(mBmsData[33]);
		mLiveData.TodayDischarging_Wh = (mBmsData[37]<<16)|(mBmsData[38]<<8)|(mBmsData[39]);
		mLiveData.Energystored_Wh = (mBmsData[34]<<16)|(mBmsData[35]<<8)|(mBmsData[36]);
		mLiveData.Status = mBmsData[30];
		mLiveData.TotalCharging_kWh = (mBmsData[41]<<16)|(mBmsData[42]<<8)|(mBmsData[43]);
		mLiveData.TotalDischarging_kWh = (mBmsData[44]<<16)|(mBmsData[45]<<8)|(mBmsData[46]);
		Cells[(mBmsData[24] - 1)].Voltage_mV = 5 * ((mBmsData[26]<<8) | mBmsData[27]);
		Cells[(mBmsData[24] - 1)].Temp_C= ((mBmsData[28]<<8) | mBmsData[29]) - 0x0114;


		// small memory usage version

		//mLiveData.VoltageTotal_mV = 5 * ((__builtin_bswap32(*((uint32_t*)(&mBmsData[0]))))>>8);

	//	sprintf(message, "Voltage %d mV \r\n", mLiveData.VoltageTotal_mV);
	//	UART_Send(message, strlen(message));

	}
	else  // we have to first allocate space for cell infos
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


/*
void LPUART1_IRQHandler(void)
{
	// Interrupt is enabled only for IDLE detection
	// Read the length of the answer from the DMA counter
	mRecLength = DMA_REC_LENGTH - DMA1_Channel3->CNDTR;

	// Disable the DMA channel
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	BMS_UART->RQR |= 0x08; //
	BMS_UART->ICR = 0x10;  // clear the IDLE flag

	mNewDataReady = 1;

}*/






void BMS1_UartTxCallback(void)
{
	mNewDataReady = 1;
	mRecLength = 1;
	//ProcessMessage();
}
