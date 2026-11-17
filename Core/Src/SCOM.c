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
#include "circbuf.h"
#include "COM.h"
#include "UI.h"
#include "RPISERP.h"
#include "RTC.h"
#include "VARS2.h"
#include "APP.h"
#include "CMD.h"
#include "MCAN.h"
#include "LOG.h"
#include <string.h>





uint8_t mRxBuffer[COM_MSGLEN];
//uint8_t mTxBuffer[COM_MSGLEN];
uint8_t mRxLength;


s_ScomTxMsg mTxMsg;


uint8_t mPcConnected;
uint16_t mPcHbTimer;
uint16_t mNsSendTimer;  // timer for sending network status
static sVarsSystem* mpSystem;   /* produced: NETWORK_STATUS */

/* RPi commands (CMD_RPI_COMMAND): fixed-window rate limit and drop log */
static uint32_t mCmdSecStart;
static uint32_t mCmdMinStart;
static uint8_t  mCmdSecCnt;
static uint8_t  mCmdMinCnt;
static uint32_t mCmdLastLogTick;
static uint8_t  mCmdLogged;
volatile uint16_t gScomCmdDropLen;     // dropped: data length != 8 (debug variable)
volatile uint16_t gScomCmdDropRate;    // dropped: over the rate limit (debug variable)
volatile uint32_t gScomCmdForwarded;   // sent to CAN 0x200 + target (debug variable)
volatile uint32_t gScomCmdLocal;       // target ELECON, handled by CMD_ProcessFrame() (debug variable)


static void ProcessMessage(void);
static void TunnelVars2Frame(uint16_t cobId, const uint8_t* data8);
static void ForwardRpiCommand(const uint8_t* data, uint8_t len);
static void LogCmdDrop(uint8_t reason, uint8_t info);

void SCOM_Init(UART_HandleTypeDef* uart, sVarsSystem* pSystem)
{
	mpSystem = pSystem;

	mRxLength = 0;
	mPcConnected = 0;

	mCmdSecStart = HAL_GetTick();
	mCmdMinStart = mCmdSecStart;
	mCmdSecCnt = 0;
	mCmdMinCnt = 0;
	mCmdLogged = 0;

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
		VarSetU32(&mpSystem->NetworkStatus, (uint16_t)COM_GetNetworkStatus(), 1);
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
			case CMD_RPI_COMMAND:
				ForwardRpiCommand(&mRxBuffer[2], (uint8_t)(mRxLength - 2));   // mRxLength > 2 here
				break;
		}

	return;
}


// RPi command (CMD_spec.md sec.3). ELECON only forwards: it does not check command ids or values.
// The CAN id is always 0x200 + (data[0] & 0x0F); nothing else from the packet selects the id.
// data = [target, cmd_lo, cmd_hi, v0, v1, v2, v3, seq], len = number of data bytes received
static void ForwardRpiCommand(const uint8_t* data, uint8_t len)
{
	uint8_t frame[8];
	uint8_t target;
	uint32_t now;

	// 1. exact length
	if (len != SCOM_CMD_DATA_LEN)
	{
		gScomCmdDropLen++;
		LogCmdDrop(SCOM_CMD_DROP_LENGTH, len);
		return;
	}

	// 2. rate limit: two fixed windows (1 s, 60 s); a dropped command does not count
	now = HAL_GetTick();
	if ((uint32_t)(now - mCmdSecStart) >= 1000u)
	{
		mCmdSecStart = now;
		mCmdSecCnt = 0;
	}
	if ((uint32_t)(now - mCmdMinStart) >= 60000u)
	{
		mCmdMinStart = now;
		mCmdMinCnt = 0;
	}
	if (mCmdSecCnt >= SCOM_CMD_MAX_PER_S || mCmdMinCnt >= SCOM_CMD_MAX_PER_MIN)
	{
		gScomCmdDropRate++;
		LogCmdDrop(SCOM_CMD_DROP_RATE, data[0]);
		return;
	}
	mCmdSecCnt++;
	mCmdMinCnt++;

	// 3. CAN command frame [cmd_lo, cmd_hi, v0, v1, v2, v3, seq, 0]
	target = data[0] & 0x0F;
	memcpy(frame, &data[1], 7);
	frame[7] = 0;
	if (target == THIS_NODE)
	{
		gScomCmdLocal++;
		(void)CMD_ProcessFrame(frame);
	}
	else
	{
		gScomCmdForwarded++;
		(void)MCAN_SendFrame((uint16_t)(CMD_NODE_COMMAND + target), frame, 8);   // fixed base, never COM_SendMessage()
	}
}


// 4. one eleLog per drop, at most one per SCOM_CMD_LOG_MIN_MS (reaches the RPi log.txt over SCOM)
static void LogCmdDrop(uint8_t reason, uint8_t info)
{
	uint8_t payload[6];
	uint32_t now = HAL_GetTick();

	if (mCmdLogged && (uint32_t)(now - mCmdLastLogTick) < SCOM_CMD_LOG_MIN_MS)
	{
		return;
	}
	mCmdLogged = 1;
	mCmdLastLogTick = now;
	// totals since start per reason, so drops suppressed by the log limit keep their reason
	payload[0] = SCOM_LOG_CMD_DROP;
	payload[1] = reason;
	payload[2] = info;
	payload[3] = (gScomCmdDropLen > 0xFFu) ? 0xFFu : (uint8_t)gScomCmdDropLen;   // saturated
	payload[4] = (uint8_t)(gScomCmdDropRate & 0xFF);
	payload[5] = (uint8_t)(gScomCmdDropRate >> 8);
	LOG_SendEvent(eleLog, payload);
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





