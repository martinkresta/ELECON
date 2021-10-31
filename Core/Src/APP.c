/*
 * APP.c
 *
 *  Created on: Aug 14, 2021
 *      Author: Martin
 *      Brief:  Main application and compile time configuration
 *
 */

#include "RTC.h"
#include "UI.h"
#include "main.h"
#include "APP.h"
#include "scheduler.h"
#include "OW.h"
#include "TEMP.h"
#include "VARS.h"
#include "SCOM.h"
#include "COM.h"
#include "AC.h"
#include "BMS1.h"
#include "BMS2.h"
#include "MPPT.h"


//#include "watchdog.h"



s_CanRxMsg rmsg;

static void ProcessMessage(s_CanRxMsg* msg);

// public methods
void APP_Init(void)
{

	sUIHwInit uihw;

	Scheduler_Init();

	OW_Init();
	TEMP_Init();
	VAR_Init();
	SCOM_Init(&huart1);
	MCAN_Init(&hcan1, THIS_NODE);
  COM_Init(THIS_NODE);
  AC_Init();
  BMS1_Init(&huart3);
  BMS2_Init(&huart2);
  MMPT_Init(&hlpuart1);
  WDG_Init(3000);



	/*Assign pins for onboard UI  */
	uihw.Led_Life.Pin = LED_Life_Pin;
	uihw.Led_Life.Port = LED_Life_GPIO_Port;
	uihw.Led_Life.Logic = eUIL_POSITIVE;

	uihw.Led_R.Pin = LED_R_Pin;
	uihw.Led_R.Port = LED_R_GPIO_Port;
	uihw.Led_R.Logic = eUIL_NEGATIVE;

	uihw.Led_G.Pin = LED_G_Pin;
	uihw.Led_G.Port = LED_G_GPIO_Port;
	uihw.Led_G.Logic = eUIL_NEGATIVE;

	uihw.Led_B.Pin = LED_B_Pin;
	uihw.Led_B.Port = LED_B_GPIO_Port;
	uihw.Led_B.Logic = eUIL_NEGATIVE;

	uihw.Buzzer.Pin = BUZZ_Pin;
	uihw.Buzzer.Port = BUZZ_GPIO_Port;
	uihw.Buzzer.Logic = eUIL_POSITIVE;

	UI_Init(&uihw);
	UI_LED_Life_SetMode(eUI_BLINKING_SLOW);


	/*Config temperature measurement*/

	// define hardware OW busses
	TEMP_AddHwBus(0,OW1_GPIO_Port, OW1_Pin);

	// assign sensors on OW1 :
	TEMP_AssignSensor(T_ELECON, VAR_TEMP_ELECON_BOARD, 0);
	TEMP_AssignSensor(T7, VAR_TEMP_OFFICE, 0);
	TEMP_AssignSensor(T310, VAR_TEMP_KIDROOM, 0);
	TEMP_AssignSensor(T304, VAR_TEMP_OUTSIDE, 0);



	/* Configure CAN streamed variables */

	COM_AddStreamedVariable(VAR_TEMP_ELECON_BOARD, 3000);
	COM_AddStreamedVariable(VAR_TEMP_OFFICE, 3000);
	COM_AddStreamedVariable(VAR_TEMP_KIDROOM, 3000);
	COM_AddStreamedVariable(VAR_TEMP_OUTSIDE, 3000);


}

void APP_Start(void)
{

//	MCAN_Start();

	UI_LED_R_SetMode(eUI_OFF);
	UI_LED_G_SetMode(eUI_OFF);
	UI_LED_B_SetMode(eUI_OFF);
	UI_Buzzer_SetMode(eUI_OFF);

	HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_RESET);


	while (1)   // endless loop
	{
		Scheduler_Check_Flag();

		// Process received COM messages

/*		s_CanRxMsg rmsg;
		while(1 == COM_GetRxMessage(&rmsg))  // process all messages in buffer
		{
				ProcessMessage(&rmsg);
		}*/

	}
}


/*Private methods*/

static void ProcessMessage(s_CanRxMsg* msg)
{
	uint16_t cmd = msg->header.StdId & 0xFF0;  // maskout nodeid
	uint8_t producer = msg->header.StdId & 0x00F;  // maskout cmd
	int16_t par1,par2,par3,par4;
	uint32_t unixtime = 0;
	par1 = msg->data[0]*0xFF + msg->data[1];
	par2 = msg->data[2]*0xFF + msg->data[3];
	par3 = msg->data[4]*0xFF + msg->data[5];
	par4 = msg->data[6]*0xFF + msg->data[7];

	switch (cmd)
	{
		case CMD_BUTTON_STATE:
			break;
		case  CMD_VAR_VALUE:
			VAR_SetVariable(par1, par2, par3);  // tbd check valid flag
			break;
		case CMD_RPI_RTC_SYNC: // set RTC time
			unixtime |= msg->data[0] << 24;
			unixtime |= msg->data[1] << 16;
			unixtime |= msg->data[2] << 8;
			unixtime |= msg->data[3];
			RTC_SetUnixTime(unixtime);
			break;
	}
	// TBD change cobID ! and put it to switch case
	if (msg->header.StdId == CMD_RPI_RTC_SYNC)
	{
		unixtime |= msg->data[0] << 24;
		unixtime |= msg->data[1] << 16;
		unixtime |= msg->data[2] << 8;
		unixtime |= msg->data[3];
		RTC_SetUnixTime(unixtime);
	}
	return;
}



/* Interrupt callbacks*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		SCOM_UartTxCallback();
	}
	else if (huart->Instance == hlpuart1.Instance)
	{
		MPPT_UartTxCallback();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart1.Instance)
	{
		SCOM_UartRxCallback();
	}
	else if (huart->Instance == hlpuart1.Instance)
	{
		MPPT_UartRxCallback(huart->RxXferSize - huart->RxXferCount);
	}
	else if (huart->Instance == huart3.Instance)
	{

		BMS1_UartRxCallback(huart->RxXferSize - huart->RxXferCount);
	}
	else if (huart->Instance == huart2.Instance)
	{
		BMS2_UartRxCallback(huart->RxXferSize - huart->RxXferCount);
	}
}

// another HAL callback when using RecieveToIdleDMA method
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == hlpuart1.Instance)
	{
		MPPT_UartRxCallback(Size);
	}
	else if (huart->Instance == huart3.Instance)
	{
		BMS1_UartRxCallback(Size);
	}
	else if (huart->Instance == huart2.Instance)
	{
		BMS2_UartRxCallback(Size);
	}

}



