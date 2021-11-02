/*
 * SHUNT.C
 *
 *  Created on: Nov 2, 2021
 *      Author: Martin
 */



#include "../Inc/SHUNT.h"

#include "VARS.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



static uint8_t mAdcData[4];
static uint8_t mTxData[10];
static uint8_t mRxData[10];



static uint8_t mNumOfMotors;
static SPI_HandleTypeDef* mSpi;
static uint8_t mSlaveHbCnt;
static uint8_t mSpiBusy;
static int64_t mRawADC;
static int32_t mIbat_mA;

static uint8_t mActiveSpiSlave;  // id of the slave which is just now selected for communication

static uint8_t IsChecksumValid(void);
static uint8_t SpiTransfer(uint8_t length, uint8_t* recdata);

// hspi has to be preconfigured by MX cube,

 void SHUNT_Init(SPI_HandleTypeDef* hspi)
 {
	 mSpi = hspi;
	 mSpiBusy = 0;


	 // HW reset
	 HAL_GPIO_WritePin(SHUNT2_CS_GPIO_Port, SHUNT1_RST_Pin, GPIO_PIN_RESET);
	 HAL_Delay(20);
	 HAL_GPIO_WritePin(SHUNT2_CS_GPIO_Port, SHUNT1_RST_Pin, GPIO_PIN_SET);
	 HAL_Delay(20);

	 // configure register to use internal analog reference 2.5V and checksum
	 mTxData[0] = SPI_CMD_WREG_MSK | ADC_REG_CONFIG1;
	 mTxData[1] = 0x00;  //  write one register
	 mTxData[2] = 0x40;  // enable checksum + enable internal reference
	 SpiTransfer(3,mRxData);
	 HAL_Delay(20);

	 // Start conversions
	 mTxData[0] = SPI_CMD_START;
	 SpiTransfer(1,mRxData);
	 HAL_Delay(20);

	 // Start continuous read mode
	 mTxData[0] = SPI_CMD_RDATAC;
	 SpiTransfer(1,mRxData);
	 HAL_Delay(3);


	 mTxData[0] = 0;
	 mTxData[1] = 0;
	 mTxData[2] = 0;
	 mTxData[3] = 0;

 }


 void SHUNT_Update_100ms()
 {
	 uint8_t i;

	// process the last readout value
	 if(IsChecksumValid())
	 {
		 mRawADC = 0;
		 mRawADC |= (mAdcData[0] & 0x7F) << 16;
		 mRawADC |= mAdcData[1] << 8;
		 mRawADC |= mAdcData[2];
		 if (mAdcData[0] & 0x80) // sign bit
		 {
			 mRawADC *= -1;
		 }
		 // Rshunt = 250uOhm   1A = 250uV
		 // OP GAIN = 50       1A = 12,5mV    200A = 2,5V
		 // V REF = 2.5V      ADC 23bit   2^23 = 8 388 608       LSB = 200/8388608 = 0,0238mA
		 mIbat_mA = (mRawADC * 200) / 8388608;

		 VAR_SetVariable(VAR_SHUNT_CURRENT_A100, (int16_t)(mIbat_mA/10),1);
	 }
	 else
	 {
		 VAR_SetVariable(VAR_SHUNT_CURRENT_A100, (int16_t)(mIbat_mA/10),0);
	 }


	// read the ADC value
	 SpiTransfer(4,mAdcData);
 }









 static uint8_t IsChecksumValid(void)
 {
	 uint8_t sum = mAdcData[0] + mAdcData[1] +  mAdcData[2] + 0x9B;
	 if (sum == mAdcData[3])
	 {
		 return 1;
	 }
	 else
	 {
		 return 0;
	 }
 }

/*SPI functions */

static uint8_t SpiTransfer(uint8_t length, uint8_t* recdata)
{
	if(mSpiBusy) 	return SPI_BUSY;
	mSpiBusy = 1;
	// activate chipselect
	HAL_GPIO_WritePin(SHUNT1_CS_GPIO_Port, SHUNT1_CS_Pin, GPIO_PIN_RESET);
	// wait unitl MISO is low (when MISO is high, data are not valid)
	while (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {}
	// do the transfer
	HAL_SPI_TransmitReceive_DMA(mSpi, mTxData, mRxData, length);
	// chipselect will be deactived in DMA TXRX complete callback
	return SPI_OK;
}

// transfer complete callback
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == SPI1)
	{
		// deactivate chipselect
		HAL_GPIO_WritePin(SHUNT1_CS_GPIO_Port, SHUNT1_CS_Pin, GPIO_PIN_SET);

		mSpiBusy = 0;
	}
}



// SPI error callback
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	//TBD
}





