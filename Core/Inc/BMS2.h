/*
 * BMS1.h
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 *       Brief: Module for one-way communication with BMS on battery pack ()
 */


#ifndef INC_BMS2_H_
#define INC_BMS2_H_


#include "main.h"
#include "BMS1.h"

#define BMS_STAT_ALLOW_CHARGE					0x01
#define BMS_STAT_ALLOW_DISCHARGE			0x02
#define BMS_STAT_COM_ERROR						0x04
#define BMS_STAT_CELL_LOW_VOLTAGE			0x08
#define BMS_STAT_CELL_HIGH_VOLTAGE		0x10
#define BMS_STAT_LOW_TEMPERATURE			0x20
#define BMS_STAT_HIGH_TEMPERATURE			0x40
#define BMS_STAT_SOC_UNKNOWN					0x80


void BMS2_Init(UART_HandleTypeDef* huart);
// Update function, to the called periodically by the scheduler
void BMS2_Update_500ms(void);
// Gets maximal cell voltage
uint16_t BMS2_GetMaxCellVoltage(void);
// Gets minimal cell voltage
uint16_t BMS2_GetMinCellVoltage(void);
// gets status of charge enable relay
uint8_t BMS2_IsChargingEnabled(void);

void BMS2_UartRxCallback(uint16_t reclength);



#endif
