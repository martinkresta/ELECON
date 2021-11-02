/*
 * SHUNT.H
 *
 *  Created on: Nov 2, 2021
 *      Author: Martin
 *      Brief : Module for communication with external ADC via SPI to monitor the battery current by shunt
 */

#ifndef INC_SHUNT_H_
#define INC_SHUNT_H_


#include "main.h"


 // SPI defines

#define 	TRANSFER_SIZE_B			5

#define		SPI_OK							0
#define 	SPI_BUSY						1


#define  SPI_CMD_WAKEUP  					0x02
#define  SPI_CMD_SLEEP	  		 		0x04
#define  SPI_CMD_RESET   					0x06
#define  SPI_CMD_START		   			0x08
#define  SPI_CMD_STOP   					0x0A
#define  SPI_CMD_RDATAC		  			0x10
#define  SPI_CMD_SDATAC  					0x11
#define  SPI_CMD_RDATA						0x12
#define  SPI_CMD_RREG_MSK				  0x20
#define  SPI_CMD_WREG_MSK   			0x40
#define  SPI_CMD_OFSCAL					  0x18
#define  SPI_CMD_GANCAL		   			0x19

#define  ADC_REG_CONFIG0					0x00
#define  ADC_REG_CONFIG1					0x01
#define  ADC_REG_CONFIG2					0x02
#define  ADC_REG_OFC0							0x03
#define  ADC_REG_OFC1							0x04
#define  ADC_REG_OFC2							0x05
#define  ADC_REG_FSC0							0x06
#define  ADC_REG_FSC1							0x07
#define  ADC_REG_FSC2							0x08



 // Init the module and pass handle to used SPI (preconfigured by MX)

void SHUNT_Init(SPI_HandleTypeDef* hspi);
void SHUNT_Update_100ms(void);



#endif /* INC_SHUNT_H_ */
