/*
 * MPPT.h
 *
 *  Created on: Oct 30, 2021
 *      Author: Martin
 *       Brief: Module for communication with Victron SmartSolar MPPT
 */


#ifndef INC_MPPT_H_
#define INC_MPPT_H_


#include "main.h"



//  VE Direct HEX protocol definitions (useful subset)

#define MPPT_FRAME_START		':'
#define MPPT_FRAME_END			'\n'
#define MPPT_CHECKSUM_RES 	0x55

#define MPPT_CMD_BOOT				0x0
#define MPPT_CMD_PING				0x1
#define MPPT_CMD_VERSION		0x3
#define MPPT_CMD_DEVID			0x4
#define MPPT_CMD_RESTART		0x6
#define MPPT_CMD_GET				0x7
#define MPPT_CMD_SET				0x8
#define MPPT_CMD_ASYNC			0xA

#define MPPT_RSP_DONE				0x1
#define MPPT_RSP_UNKNOWN		0x3
#define MPPT_RSP_ERR				0x4
#define MPPT_RSP_PING				0x5
#define MPPT_RSP_GET				0x7
#define MPPT_RSP_SET				0x8

#define MPPT_FLAG_UNKNOWN_ID  		0x01
#define MPPT_FLAG_NOT_SUPPORTED		0x02
#define MPPT_FLAG_PARAM_ERROR			0x04

#define MPPT_MODE_ON							0x01
#define MPPT_MODE_OFF							0x04

#define MPPT_STATE_NOT_CHARGING		0x00
#define MPPT_STATE_FAULT					0x02
#define MPPT_STATE_BULK						0x03
#define MPPT_STATE_ABSORPTION			0x04
#define MPPT_STATE_FLOAT					0x05
#define MPPT_STATE_EQUALIZE				0x07

#define MPPT_OFFREASON_NOSUN			0x00
#define MPPT_OFFREASON_SOFTSWITCH	0x02
#define MPPT_OFFREASON_REMOTE			0x03
#define MPPT_OFFREASON_INTERNAL		0x04
#define MPPT_OFFREASON_CREDIT			0x05
#define MPPT_OFFREASON_BMS				0x06

#define MPPT_REG_MAX_CHARGING_CURRENT 		0xEDF0
#define MPPT_REG_TEMP									 		0xEDDB
#define MPPT_REG_CHARGER_CURRENT			 		0xEDD7
#define MPPT_REG_CHARGER_VOLTAGE			 		0xEDD5
#define MPPT_REG_YIELD_TODAY					 		0xEDD3
#define MPPT_REG_MAX_POWER_TODAY			 		0xEDD2

#define MPPT_REG_SOLAR_POWER							0xEDBC
#define MPPT_REG_SOLAR_VOLTAGE						0xEDBB
#define MPPT_REG_SOLAR_CURRENT						0xEDBD
#define MPPT_REG_SOLAR_MAX_VOLTAGE				0xEDB8

#define DMA_LENGTH 50 // maximal allowed length of DMA transfer


typedef enum
{
	etnone,
	etu8,
	etu16,
	etu32,
	ets16,
	ets32
}eValType;

typedef struct
{
	uint8_t  cmd;
	uint16_t reg;
	uint32_t value;
	eValType datatype;
}sTxMsg;

typedef struct
{
	uint8_t  cmd;
	uint16_t reg;
	uint32_t value;
	uint8_t flags;
}sRxMsg;


void MPPT_Init(UART_HandleTypeDef* huart);
// Update function, to the called periodically by the scheduler
void MPPT_Update_100ms(void);


void MPPT_UartRxCallback(uint16_t reclength);
void MPPT_UartTxCallback(void);

void MPPT_MidnightNow(void);




#endif
