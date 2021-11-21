/*
 * pwrout.c - module for control of power outputs with PWM capability.
 * 					- based on timer OC channels
 *
 *  Created on: Apr 29, 2021
 *      Author: Martin
 */

#ifndef INC_PWROUT_H_
#define INC_PWROUT_H_


#include "main.h"

#define  NUM_OF_OUTPUTS		3

#define 	OUT1  0
#define 	OUT2  1
#define 	OUT3  2
#define 	OUT4  3


typedef struct
{
	TIM_HandleTypeDef*					Timer;
  uint32_t										Channel;
  uint8_t											DutyCycle;
}sPwrOut;


void PWROUT_Init(void);
void PWROUT_SetPct(uint8_t output,uint8_t dutyCycle);

uint8_t PWROUT_GetPct(uint8_t output);





#endif /* INC_PWROUT_H_ */
