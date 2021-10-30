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


void BMS2_Init(UART_HandleTypeDef* huart);
// Update function, to the called periodically by the scheduler
void BMS2_Update_500ms(void);


void BMS2_UartRxCallback(uint16_t reclength);



#endif
