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


#define SCOM_BAUDRATE 57600

#define COM_MSGLEN	10

#define SCOM_TX_MSG_BUFLEN	255

#define NUM_OF_SCAN_VARS	180



// all commands has to fit to 11-bit value .  0x0000 - 0x07FF
// PC application commands   range 0x
#define  CMD_GET_DEV_ID  					0x710
#define  CMD_GET_STATUS  					0x712
#define  CMD_READ_VAR_REQUEST  		0x713
#define  CMD_TM_SET_POWER_OUTPUTS 0x721
#define  CMD_TM_SET_ELV 					0x722
#define  CMD_TM_SET_AV  					0x723
#define  CMD_TM_SET_SERVOVALVES  	0x724
#define  CMD_TM_SET_PUMPS 				0x725

#define  CMD_MASTER_HB  					0x777
#define  CMD_NETWORK_STATUS				0x765


#define  CMD_TM_DEV_ID  					0x210
#define  CMD_TM_STATUS  					0x212
#define  CMD_TM_VAR_VALUE  				0x221

#define  CMD_RPI_VAR_VALUE  			0x50
#define  CMD_SET_VAR_VALUE 			 	0x120
#define  CMD_RPI_RTC_SYNC  				0x51

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
	uint8_t data[COM_MSGLEN];
}s_ScomTxMsg;



void SCOM_Init(UART_HandleTypeDef* uart);
void SCOM_Update_10ms(void);

void SCOM_Transmit(void);

void SCOM_UartTxCallback();
void SCOM_UartRxCallback();





#endif /* INC_SCOM_H_ */
