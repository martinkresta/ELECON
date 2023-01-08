/*
 * SCOM.c
 *
 *  Created on: Jul 13, 2021
 *      Author: Martin
 *      Brief:  Universal serial communication protocol between device and computer.
 *     	The handler of used UART is inserted as parameter to SCOM_Init
 */

#include "main.h"
#include "APP.h"
#include "SCOM.h"
#include "VARS.h"
#include "circbuf.h"
#include "COM.h"
#include "UI.h"
#include "RPISERP.h"
#include "RTC.h"


typedef struct
{
	uint8_t enable;
	uint16_t varId;
	uint16_t sendPeriod;  // 10ms
	uint16_t timer;
	void* next;
}sScanVariable;




s_ScomTxMsg mTxMsg;

sScanVariable mScanList[NUM_OF_SCAN_VARS];

uint8_t mPcConnected;
uint16_t mPcHbTimer;
uint16_t mNsSendTimer;  // timer for sending network status


static void UpdateScanList(uint16_t varId, uint16_t period);
static void SendVariable(uint16_t id);
static void InitPcScanList(void);
static void Send(s_ScomTxMsg msg, uint8_t bytesToSend);
static void ProcessMessage(void);

void SCOM_Init(UART_HandleTypeDef* uart)
{

	//ComUart = uart;
	mPcConnected = 0;
	InitPcScanList();

	RSP_Init(uart, &hdma_usart1_tx.Instance, &hdma_usart1_rx.Instance);



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

//	SCOM_Transmit();  // transmit physically;
}

/* private methods */


// Compile-time initialization of list of variables periodically sent toi the PC app
static void InitPcScanList(void)
{

	UpdateScanList(VAR_BAT_SOC, 1000);
	UpdateScanList(VAR_BAT_VOLTAGE_V10, 1000);
	UpdateScanList(VAR_LOAD_A100, 1000);
	UpdateScanList(VAR_CHARGING_A10, 1000);
	UpdateScanList(VAR_BAT_CURRENT_A10, 1000);
	UpdateScanList(VAR_CONS_TODAY_WH, 3000);
	UpdateScanList(VAR_BAT_ENERGY_WH, 3000);
	UpdateScanList(VAR_LOAD_W, 1000);
	UpdateScanList(VAR_SHUNT_CURRENT_A100, 1000);


	UpdateScanList(VAR_BMS1_SOC, 1000);
	UpdateScanList(VAR_BMS1_CURRENT_A10, 3000);
	UpdateScanList(VAR_BMS1_VOLTAGE_V10, 3000);
	UpdateScanList(VAR_BMS1_ENERGY_STORED_WH, 3000);
	UpdateScanList(VAR_BMS1_TODAY_ENERGY_WH, 3000);

	UpdateScanList(VAR_BMS2_SOC, 1000);
	UpdateScanList(VAR_BMS2_CURRENT_A10, 3000);
	UpdateScanList(VAR_BMS2_VOLTAGE_V10, 3000);
	UpdateScanList(VAR_BMS2_ENERGY_STORED_WH, 3000);
	UpdateScanList(VAR_BMS2_TODAY_ENERGY_WH, 3000);

	UpdateScanList(VAR_MPPT_BAT_CURRENT_A10, 1000);
	UpdateScanList(VAR_MPPT_BAT_VOLTAGE_V100, 1000);
	UpdateScanList(VAR_MPPT_YIELD_TODAY_10WH, 3000);
	UpdateScanList(VAR_MPPT_MAX_TODAY_W, 3000);
	UpdateScanList(VAR_MPPT_SOLAR_POWER_W, 1000);
	UpdateScanList(VAR_MPPT_SOLAR_VOLTAGE_V100, 1000);
	UpdateScanList(VAR_MPPT_SOLAR_CURRENT_A10, 3000);
	UpdateScanList(VAR_MPPT_SOLAR_MAX_VOLTAGE_V100, 3000);
	UpdateScanList(VAR_MPPT_MAX_BAT_CURRENT_A10, 3000);
	UpdateScanList(VAR_MPPT_TEMP_C, 3000);

	UpdateScanList(VAR_EL_HEATER_STATUS, 3000);
	UpdateScanList(VAR_EL_HEATER_POWER, 3000);
	UpdateScanList(VAR_EL_HEATER_CURRENT, 3000);
	UpdateScanList(VAR_EL_HEATER_CONS, 3000);

	UpdateScanList(VAR_FLOW_COLD, 3000);
	UpdateScanList(VAR_FLOW_HOT, 3000);
	UpdateScanList(VAR_CONS_COLD, 3000);
	UpdateScanList(VAR_CONS_HOT, 3000);

	UpdateScanList(VAR_HEAT_TOTAL_WH, 3000);
	UpdateScanList(VAR_HEAT_HEATING_WH, 3000);

	UpdateScanList(VAR_CONS_AC300_WH, 3000);
	UpdateScanList(VAR_CONS_AC3KW_WH, 3000);
	UpdateScanList(VAR_CONS_AC5KW_WH, 3000);
	UpdateScanList(VAR_CONS_FRIDGE_WH, 3000);
	UpdateScanList(VAR_CONS_KITCHEN_WH, 3000);
	UpdateScanList(VAR_CONS_WASCHMACHINE_WH, 3000);
	UpdateScanList(VAR_CONS_OTHER_WH, 3000);
	UpdateScanList(VAR_CONS_TECHM_WH, 3000);

	UpdateScanList(VAR_POW_AC300_W, 3000);
	UpdateScanList(VAR_POW_AC3KW_W, 3000);
	UpdateScanList(VAR_POW_AC5KW_W, 3000);
	UpdateScanList(VAR_POW_FRIDGE_W, 3000);
	UpdateScanList(VAR_POW_KITCHEN_W, 3000);
	UpdateScanList(VAR_POW_WASCHMACHINE_W, 3000);
	UpdateScanList(VAR_POW_OTHER_W, 3000);
	UpdateScanList(VAR_POW_TECHM_W, 3000);


	UpdateScanList(VAR_BOILER_POWER, 3000);
	UpdateScanList(VAR_BOILER_HEAT, 3000);
	UpdateScanList(VAR_TEMP_BOILER, 3000);
	UpdateScanList(VAR_TEMP_BOILER_IN, 3000);
	UpdateScanList(VAR_TEMP_BOILER_OUT, 3000);
	UpdateScanList(VAR_TEMP_TANK_IN_H, 3000);
	UpdateScanList(VAR_TEMP_TANK_OUT_H, 3000);
	UpdateScanList(VAR_TEMP_TANK_1, 3000);
	UpdateScanList(VAR_TEMP_TANK_2, 3000);
	UpdateScanList(VAR_TEMP_TANK_3, 3000);
	UpdateScanList(VAR_TEMP_TANK_4, 3000);
	UpdateScanList(VAR_TEMP_TANK_5, 3000);
	UpdateScanList(VAR_TEMP_TANK_6, 3000);
	UpdateScanList(VAR_TEMP_WALL_IN, 3000);
	UpdateScanList(VAR_TEMP_WALL_OUT, 3000);
	UpdateScanList(VAR_TEMP_BOILER_EXHAUST, 3000);
	UpdateScanList(VAR_TEMP_RAD_H, 3000);
	UpdateScanList(VAR_TEMP_RAD_C, 3000);
	UpdateScanList(VAR_TEMP_TANK_IN_C, 3000);
	UpdateScanList(VAR_TEMP_TANK_OUT_C, 3000);


	UpdateScanList(VAR_TEMP_TECHM_BOARD, 3000);
	UpdateScanList(VAR_TEMP_IOBOARD_D, 3000);
	UpdateScanList(VAR_TEMP_IOBOARD_U, 3000);
	UpdateScanList(VAR_TEMP_ELECON_BOARD, 3000);
	UpdateScanList(VAR_TEMP_DOWNSTAIRS, 3000);
	UpdateScanList(VAR_TEMP_OFFICE, 3000);
	UpdateScanList(VAR_TEMP_KIDROOM, 3000);
	UpdateScanList(VAR_TEMP_OUTSIDE, 3000);

	UpdateScanList(VAR_METEO_WIND_BURST, 1000);
	UpdateScanList(VAR_METEO_WIND_AVG, 1000);
	UpdateScanList(VAR_METEO_WIND_POW, 5000);
	UpdateScanList(VAR_METEO_WIND_ENERGY, 5000);

	UpdateScanList(VAR_TEMP_RECU_FC, 3000);
	UpdateScanList(VAR_TEMP_RECU_FH, 3000);
	UpdateScanList(VAR_TEMP_RECU_WH, 3000);
	UpdateScanList(VAR_TEMP_RECU_WC, 3000);
	UpdateScanList(VAR_RH_RECU_FH, 3000);
	UpdateScanList(VAR_RH_RECU_WH, 3000);
	UpdateScanList(VAR_CO2_RECU, 3000);
	UpdateScanList(VAR_DP_RECU_F, 3000);
	UpdateScanList(VAR_DP_RECU_W, 3000);
	UpdateScanList(VAR_RECU_FAN_F, 3000);
	UpdateScanList(VAR_RECU_FAN_W, 3000);
	UpdateScanList(VAR_CURR_RECU_A, 3000);


	UpdateScanList(VAR_BMS1_CELL1_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL2_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL3_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL4_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL5_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL6_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL7_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL8_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL9_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL10_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL11_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL12_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL13_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL14_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL15_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL16_MV, 3000);
	UpdateScanList(VAR_BMS1_CELL1_C, 3000);
	UpdateScanList(VAR_BMS1_CELL2_C, 3000);
	UpdateScanList(VAR_BMS1_CELL3_C, 3000);
	UpdateScanList(VAR_BMS1_CELL4_C, 3000);
	UpdateScanList(VAR_BMS1_CELL5_C, 3000);
	UpdateScanList(VAR_BMS1_CELL6_C, 3000);
	UpdateScanList(VAR_BMS1_CELL7_C, 3000);
	UpdateScanList(VAR_BMS1_CELL8_C, 3000);
	UpdateScanList(VAR_BMS1_CELL9_C, 3000);
	UpdateScanList(VAR_BMS1_CELL10_C, 3000);
	UpdateScanList(VAR_BMS1_CELL11_C, 3000);
	UpdateScanList(VAR_BMS1_CELL12_C, 3000);
	UpdateScanList(VAR_BMS1_CELL13_C, 3000);
	UpdateScanList(VAR_BMS1_CELL14_C, 3000);
	UpdateScanList(VAR_BMS1_CELL15_C, 3000);
	UpdateScanList(VAR_BMS1_CELL16_C, 3000);

	UpdateScanList(VAR_BMS2_CELL1_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL2_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL3_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL4_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL5_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL6_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL7_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL8_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL9_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL10_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL11_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL12_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL13_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL14_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL15_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL16_MV, 3000);
	UpdateScanList(VAR_BMS2_CELL1_C, 3000);
	UpdateScanList(VAR_BMS2_CELL2_C, 3000);
	UpdateScanList(VAR_BMS2_CELL3_C, 3000);
	UpdateScanList(VAR_BMS2_CELL4_C, 3000);
	UpdateScanList(VAR_BMS2_CELL5_C, 3000);
	UpdateScanList(VAR_BMS2_CELL6_C, 3000);
	UpdateScanList(VAR_BMS2_CELL7_C, 3000);
	UpdateScanList(VAR_BMS2_CELL8_C, 3000);
	UpdateScanList(VAR_BMS2_CELL9_C, 3000);
	UpdateScanList(VAR_BMS2_CELL10_C, 3000);
	UpdateScanList(VAR_BMS2_CELL11_C, 3000);
	UpdateScanList(VAR_BMS2_CELL12_C, 3000);
	UpdateScanList(VAR_BMS2_CELL13_C, 3000);
	UpdateScanList(VAR_BMS2_CELL14_C, 3000);
	UpdateScanList(VAR_BMS2_CELL15_C, 3000);
	UpdateScanList(VAR_BMS2_CELL16_C, 3000);

}

//returns 0 when OK, 1 if transceiver is busy
static void Send(s_ScomTxMsg msg, uint8_t bytesToSend)
{
  RSP_Send((uint8_t*) &msg, bytesToSend);
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

	msg.data[0] = MSG_START_B1;     // message START
	msg.data[1] = MSG_START_B2;
	msg.data[2] = CMD_TM_VAR_VALUE >> 8;
	msg.data[3] = CMD_TM_VAR_VALUE & 0xFF;
	msg.data[4] = id >> 8;
	msg.data[5] = id  & 0xFF;
	msg.data[6] = tmp >> 8;
	msg.data[7] = tmp & 0xFF;
	msg.data[8] = validflag >> 8;
	msg.data[9] = validflag & 0xFF;
	Send(msg, COM_MSGLEN);
}

static void ProcessMessage(void)
{

    uint8_t rxData[COM_MSGLEN];
    uint8_t length;
		uint16_t varId, sendPeriod;
		uint16_t id;
		uint32_t unixtime = 0;

		if (false == RSP_GetMessage(rxData, &length))
		{
		   // no data available in the receive fifo buffer
		  return;
		}

		id = (rxData[0]<<8) | rxData[1];

		uint16_t data1, data2, data3, data4;
		data1 = (rxData[2]<<8) | rxData[3];
		data2 = (rxData[4]<<8) | rxData[5];
		data3 = (rxData[6]<<8) | rxData[7];
		data4 = (rxData[8]<<8) | rxData[9];

		switch (id )  // message ID
		{
			case CMD_MASTER_HB:
				mPcConnected = 1;
				mPcHbTimer = 0;
				break;
			case CMD_READ_VAR_REQUEST:
				varId = data1;
				sendPeriod = data2;
				UpdateScanList(varId, sendPeriod);
				SendVariable(varId);
				break;
			case CMD_RPI_RTC_SYNC:
				// forward to can
			  rxData[6] = 0;
			  rxData[7] = 0;
			  rxData[8] = 0;
			  rxData[9] = 0;
				COM_SendMessage(CMD_RTC_SYNC,&(rxData[2]),8);
				// and set also the RTC here
				unixtime |= rxData[2] << 24;
				unixtime |= rxData[3] << 16;
				unixtime |= rxData[4] << 8;
				unixtime |= rxData[5];
				RTC_SetUnixTime(unixtime);
				APP_RpiHeartbeat();
				break;
			case CMD_SET_VAR_VALUE:
				VAR_SetVariable(data1 & 0x7FFF, data2, ((data1 & 0x8000)? 0 : 1));
				break;
		}


	return;
}

