/*
 * BMS1.h
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 *       Brief: Module for one-way communication with BMS on battery pack ()
 */


#ifndef INC_BMS1_H_
#define INC_BMS1_H_


#include "main.h"



void BMS_Init(void);
// Update function, to the called periodically by the scheduler
void BMS_Update_500ms(void);


void BMS1_UartRxCallback(void);



#endif
