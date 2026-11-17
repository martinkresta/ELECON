/*
 * SCOM.h
 *
 *  Created on: Jul 13, 2021
 *      Author: Martin
 *      Brief:  Universal serial communication protocol between device and computer.
 */

#ifndef INC_SCOM_H_
#define INC_SCOM_H_

#include "main.h"
#include "uha_vars_gen.h"   /* sVarsSystem */


#define SCOM_BAUDRATE 57600

#define COM_MSGLEN	10

#define SCOM_TX_MSG_BUFLEN	255




// all commands has to fit to 11-bit value .  0x0000 - 0x07FF
// PC application commands   range 0x
#define  CMD_GET_DEV_ID  					0x710
#define  CMD_GET_STATUS  					0x712
#define  CMD_TM_SET_POWER_OUTPUTS 0x721
#define  CMD_TM_SET_ELV 					0x722
#define  CMD_TM_SET_AV  					0x723
#define  CMD_TM_SET_SERVOVALVES  	0x724
#define  CMD_TM_SET_PUMPS 				0x725

#define  CMD_MASTER_HB  					0x777
#define  CMD_NETWORK_STATUS				0x765


#define  CMD_TM_DEV_ID  					0x210
#define  CMD_TM_STATUS  					0x212

#define  CMD_RPI_VAR_VALUE  			0x50
#define  CMD_RPI_RTC_SYNC  				0x51
#define  CMD_RPI_COMMAND  				0x52   // RPi command -> CAN 0x200 + target (CMD_spec.md sec.2, 3)

// RPi command forwarding (CMD_spec.md sec.3)
#define  SCOM_CMD_DATA_LEN        8       // exact data length of a CMD_RPI_COMMAND packet
#define  SCOM_CMD_MAX_PER_S       10      // rate limit: commands per 1 s window
#define  SCOM_CMD_MAX_PER_MIN     20      // rate limit: commands per 60 s window
#define  SCOM_CMD_LOG_MIN_MS      1000    // at most one drop LOG per second (drops in between are counted)
#define  SCOM_LOG_CMD_DROP        0x30    // eleLog payload: [0x30, reason, info, len_drops (u8 sat.), rate_drops u16 LE]
                                          //   drop counts = totals since start (not since the last LOG)
#define  SCOM_CMD_DROP_LENGTH     1       //   reason 1: data length != 8, info = received data length
#define  SCOM_CMD_DROP_RATE       2       //   reason 2: over 10/s or 20/min, info = target byte

#define  MSG_START_B1							0x7F
#define  MSG_START_B2							0xAA



#define TEMP_READ_ROM	 0x51
#define TEMP_CONVERT	 0x52
#define TEMP_GET_TEMP	 0x53
#define TEMP_READ_TEMP	 0x54


#define PC_HB_TIMEOUT			3000
#define NS_SEND_PERIOD		5000


typedef struct
{
	uint8_t data[10];
}s_ScomTxMsg;



/* pSystem: produced group Vars2.System (NETWORK_STATUS, tunnel only) */
void SCOM_Init(UART_HandleTypeDef* uart, sVarsSystem* pSystem);
void SCOM_Update_10ms(void);

void SCOM_SendMsg(uint8_t* data, uint8_t size);
void SCOM_SendLogMsg(uint8_t* data, uint8_t size);

void SCOM_Transmit(void);

void SCOM_UartTxCallback();
void SCOM_UartRxCallback();





#endif /* INC_SCOM_H_ */
