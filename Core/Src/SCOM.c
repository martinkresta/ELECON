/*
 * SCOM.c
 *
 *  Created on: Jul 13, 2021
 *      Author: Martin
 *      Brief:  Universal serial communication protocol between device and computer.
 *     	The handler of used UART is inserted as parameter to SCOM_Init
 */

#include "main.h"
#include "SCOM.h"
#include "VARS.h"
#include "circbuf.h"
#include "COM.h"
#include "UI.h"
#include "RPISERP.h"
#include "RTC.h"
#include "VARS2.h"
#include "APP.h"
#include <string.h>





uint8_t mRxBuffer[COM_MSGLEN];
//uint8_t mTxBuffer[COM_MSGLEN];
uint8_t mRxLength;


s_ScomTxMsg mTxMsg;


uint8_t mPcConnected;
uint16_t mPcHbTimer;
uint16_t mNsSendTimer;  // timer for sending network status


static void ProcessMessage(void);
static void TunnelVars2Frame(uint16_t cobId, const uint8_t* data8);

void SCOM_Init(UART_HandleTypeDef* uart)
{

	mRxLength = 0;
	mPcConnected = 0;

	RSP_Init(uart,hdma_usart1_tx.Instance, hdma_usart1_rx.Instance);

	// VARS2 tunnel to the RPi (spec sec.7): every VARS2 frame received from / sent to CAN
	VARS2_SetFrameHook(TunnelVars2Frame);

}


void SCOM_Update_10ms(void)
{

	// check PC heartbeat
	mPcHbTimer += 10;
	if (mPcHbTimer > PC_HB_TIMEOUT)
	{
		mPcConnected = 0;    // heartbeat timeout elapsed
	}


	// network status every NS_SEND_PERIOD; reaches the RPi as VARS2 NETWORK_STATUS (0x184, tunnel only)
	mNsSendTimer += 10;
	if (mNsSendTimer > NS_SEND_PERIOD)
	{
		mNsSendTimer = 0;
		VAR_SetVariable(VAR_NETWORK_STATUS,COM_GetNetworkStatus(), 1);
	}


	// ELECON-local VARS2 variables that are not on CAN: tunnel only
	VARS2_UpdateLocal_10ms(TunnelVars2Frame);

	RSP_TransmitFromFifo();   // transmit physically;
	//SCOM_Transmit();

	if(true == RSP_GetMessage(mRxBuffer, &mRxLength))
	{
	  if (mRxLength > 2)
	  {
	    ProcessMessage();
	  }
	}
}


void SCOM_SendMsg(uint8_t* data, uint8_t size)
{
  RSP_Send(data, size);
}

void SCOM_SendLogMsg(uint8_t* data, uint8_t size)
{
  uint8_t scomLogData[10];
  memset(scomLogData,0,10);

  scomLogData[0] = CMD_LOG_MSG >> 8;
  scomLogData[1] = CMD_LOG_MSG & 0xFF;
  memcpy(&(scomLogData[2]), data,size);
  SCOM_SendMsg(scomLogData,10);
}


/* private methods */


// VARS2 tunnel: RPISERP packet id = CAN COB-ID (0x180 + producer), data = the 8 CAN data bytes unchanged
static void TunnelVars2Frame(uint16_t cobId, const uint8_t* data8)
{
	uint8_t msg[10];

	msg[0] = cobId >> 8;
	msg[1] = cobId & 0xFF;
	memcpy(&msg[2], data8, 8);
	RSP_Send(msg, 10);
}


static void ProcessMessage(void)
{
		uint16_t id = (mRxBuffer[0]<<8) | mRxBuffer[1];
		uint32_t unixtime = 0;

		uint16_t data1, data2;// data3, data4;
		data1 = (mRxBuffer[2]<<8) | mRxBuffer[3];
		data2 = (mRxBuffer[4]<<8) | mRxBuffer[5];
	//	data3 = (mRxBuffer[6]<<8) | mRxBuffer[7];
	//	data4 = (mRxBuffer[8]<<8) | mRxBuffer[9];

		switch (id )  // message ID
		{
			case CMD_MASTER_HB:
				mPcConnected = 1;
				mPcHbTimer = 0;
				break;
			case CMD_RPI_RTC_SYNC:
				// forward to can
				mRxBuffer[6] = 0;
				mRxBuffer[7] = 0;
				mRxBuffer[8] = 0;
				mRxBuffer[9] = 0;
				COM_SendMessage(CMD_RTC_SYNC,&(mRxBuffer[2]),8);
				// and set also the RTC here
				unixtime |= mRxBuffer[2] << 24;
				unixtime |= mRxBuffer[3] << 16;
				unixtime |= mRxBuffer[4] << 8;
				unixtime |= mRxBuffer[5];
				RTC_SetUnixTime(unixtime);
				APP_RpiHeartbeat();
				break;
			case CMD_SET_VAR_VALUE:
				VAR_SetVariable(data1 & 0x7FFF, data2, ((data1 & 0x8000)? 0 : 1));
				break;
		}

	return;
}




/* Interrupt callbacks */
void SCOM_UartTxCallback(void)
{
  while(1)
  {}
  // SW Error

}


void SCOM_UartRxCallback(void)
{
  while(1)
  {}
    // SW Error

	//ProcessMessage();
}





