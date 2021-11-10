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


typedef struct
{
	uint8_t enable;
	uint16_t varId;
	uint16_t sendPeriod;  // 10ms
	uint16_t timer;
	void* next;
}sScanVariable;



uint8_t mRxBuffer[COM_MSGLEN];
//uint8_t mTxBuffer[COM_MSGLEN];
uint8_t mRxLength, mNewDataReady, mTxBusy;

CB_handle mTxMsgBuffer;

UART_HandleTypeDef* ComUart;
sScanVariable mScanList[NUM_OF_SCAN_VARS];

uint8_t mPcConnected;
uint16_t mPcHbTimer;
uint16_t mNsSendTimer;  // timer for sending network status


static void UpdateScanList(uint16_t varId, uint16_t period);
static void SendVariable(uint16_t id);
static void InitPcScanList(void);
static uint8_t Send(s_ScomTxMsg msg);
static void ProcessMessage(void);

void SCOM_Init(UART_HandleTypeDef* uart)
{

	ComUart = uart;
	mRxLength = 0;
	mNewDataReady = 0;
	mTxBusy = 0;
	mPcConnected = 0;
	InitPcScanList();

	mTxMsgBuffer = CB_Create(sizeof(s_ScomTxMsg),SCOM_TX_MSG_BUFLEN);
	if (mTxMsgBuffer == NULL)
	{
		// TBD error
	}

	// enable receiver
	HAL_UART_Receive_DMA(ComUart, mRxBuffer, 10);

}


void SCOM_Update_10ms(void)
{

	// check PC heartbeat
	mPcHbTimer += 10;
	if (mPcHbTimer > PC_HB_TIMEOUT)
	{
		mPcConnected = 0;    // heartbeat timeout elapsed
	}


	//if (mPcConnected)  // send variables only if PC is connected
	if (1)
	{
		int i;

		// send network status every NS_SEND_PERIOD
		mNsSendTimer += 10;
		if (mNsSendTimer > NS_SEND_PERIOD)
		{
			mNsSendTimer = 0;
			VAR_SetVariable(VAR_NETWORK_STATUS,COM_GetNetworkStatus(), 1);
			SendVariable(VAR_NETWORK_STATUS);
		}


		// do the timing of the scanned variables
		for(i = 0; i < NUM_OF_SCAN_VARS; i++)
		{
			mScanList[i].timer+=10;
		}

    // and send them if its time
		for(i = 0; i < NUM_OF_SCAN_VARS; i++)
		{
			if (mScanList[i].enable == 1 && mScanList[i].sendPeriod != 0)
			{
				if (mScanList[i].timer >= mScanList[i].sendPeriod)
				{
					SendVariable(mScanList[i].varId);
					mScanList[i].timer = 0;
					break;  // send just 1 value every 10ms
				}
			}
		}
	}

	SCOM_Transmit();  // transmit physically;
}

/* private methods */


// Compile-time initialization of list of variables periodically sent toi the PC app
static void InitPcScanList(void)
{
	UpdateScanList(VAR_TEMP_ELECON_BOARD, 3000);
	UpdateScanList(VAR_TEMP_OFFICE, 3000);
	UpdateScanList(VAR_TEMP_KIDROOM, 3000);
	UpdateScanList(VAR_TEMP_OUTSIDE, 3000);
}

//returns 0 when OK, 1 if transceiver is busy
static uint8_t Send(s_ScomTxMsg msg)
{

	// instert to Tx buffer
	CB_Put(mTxMsgBuffer,(uint8_t*) &msg);
}

static void UpdateScanList(uint16_t varId, uint16_t period)
{
	// go thru the list to find if entry already exists
	int i;
	for(i = 0; i < NUM_OF_SCAN_VARS; i++)
	{
		if(mScanList[i].varId == varId)
		{
			if(period != 0)
			{
				mScanList[i].sendPeriod = period;
				mScanList[i].enable = 1;
				return;
			}
			else
			{
				mScanList[i].sendPeriod = 0;
				mScanList[i].enable = 0;
			}
		}
	}

	// if not add variable to the list
	for(i = 0; i < NUM_OF_SCAN_VARS; i++)
	{
		if(mScanList[i].enable == 0)
		{
			if(period != 0)
			{
				mScanList[i].varId = varId;
				mScanList[i].sendPeriod = period;
				mScanList[i].enable = 1;
				return;
			}
		}
	}
}

static void SendVariable(uint16_t id)
{
	uint16_t invalid = 0;
	uint16_t validflag = 0;
	s_ScomTxMsg msg;
	int16_t tmp = VAR_GetVariable(id, &invalid);
	validflag = (invalid == INVALID_FLAG ? 0 : 1);
	msg.data[0] = CMD_TM_VAR_VALUE >> 8;
	msg.data[1] = CMD_TM_VAR_VALUE & 0xFF;
	msg.data[2] = id >> 8;
	msg.data[3] = id  & 0xFF;
	msg.data[4] = tmp >> 8;
	msg.data[5] = tmp & 0xFF;
	msg.data[6] = validflag >> 8;
	msg.data[7] = validflag & 0xFF;
	msg.data[8] = 0;
	msg.data[9] = 0;
	Send(msg);
}

static void ProcessMessage(void)
{
		uint16_t varId, sendPeriod;
		uint16_t id = (mRxBuffer[0]<<8) | mRxBuffer[1];

		uint16_t data1, data2, data3, data4;
		data1 = (mRxBuffer[2]<<8) | mRxBuffer[3];
		data2 = (mRxBuffer[4]<<8) | mRxBuffer[5];
		data3 = (mRxBuffer[6]<<8) | mRxBuffer[7];
		data4 = (mRxBuffer[8]<<8) | mRxBuffer[9];

		switch (id )  // message ID
		{
			case CMD_MASTER_HB:
				mPcConnected = 1;
				mPcHbTimer = 0;
				break;
			case CMD_READ_VAR_REQUEST:
				varId = (mRxBuffer[2]<<8) | mRxBuffer[3];
				sendPeriod = (mRxBuffer[4]<<8) | mRxBuffer[5];
				UpdateScanList(varId, sendPeriod);
				SendVariable(varId);
				break;
			case CMD_TM_SET_ELV:
			//	DO_SetElv(data1);
				break;
			case CMD_SET_VAR_VALUE:
				VAR_SetVariable(data1 & 0x7FFF, data2, ((data1 & 0x8000)? 0 : 1));
				break;
		}

	HAL_UART_Receive_DMA(ComUart, mRxBuffer, 10);
	return;
}



// function to be called periodically at a rate of transmitting SCOM messages. (for example every 5 ms)
// One SCOM message is is sent if TX buffer is not empty
void SCOM_Transmit(void)
{
	s_ScomTxMsg TxMsg;


	if  (0 == CB_Probe(mTxMsgBuffer,(uint8_t*)&TxMsg) && (mTxBusy == 0))  // fetch the message from the buffer
	{
		if (HAL_OK == HAL_UART_Transmit_DMA(ComUart, TxMsg.data, COM_MSGLEN))  // transmit the message
		{
			CB_Remove(mTxMsgBuffer);  // remove the message from the buffer only if transmission was succesfull
		}
	}
}


/* Interrupt callbacks */
void SCOM_UartTxCallback(void)
{
	mTxBusy = 0;
	SCOM_Transmit();   // automatically transmit next messages
}


void SCOM_UartRxCallback(void)
{
	mNewDataReady = 1;
	mRxLength = 1;
	ProcessMessage();
}





