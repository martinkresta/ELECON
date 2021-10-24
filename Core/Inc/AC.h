/*
 * REL.h
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 *      Brief : Basic On-Off controll of three DCAC converters. Automatic control based on RTC
 */

#ifndef INC_AC_H_
#define INC_AC_H_

#include "main.h"

typedef enum
{
	acs_OFF,
	acs_ON,
}
eACState;



void AC_Init(void);

void AC_Update_1s(void);

void AC_300W(eACState state);

void AC_3kW(eACState state);

void AC_5kW(eACState state);



#endif /* INC_AC_H_ */
