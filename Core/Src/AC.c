/*
 * REL.c
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 */


#include "AC.h"
#include "RTC.h"



void AC_Init(void)
{
	AC_300W(acs_ON);
	AC_3kW(acs_OFF);
	AC_5kW(acs_OFF);
}

void AC_Update_1s(void)
{

	sDateTime now = RTC_GetTime();
	if(now.Hour >= 5  && now.Hour <= 23)
	{
		AC_3kW(acs_ON);
		AC_5kW(acs_ON);
	}
	else
	{
		AC_3kW(acs_OFF);
		AC_5kW(acs_OFF);
	}

}

void AC_300W(eACState state)
{
	if (state == acs_ON)
	{
		HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_RESET);
	}
}

void AC_3kW(eACState state)
{
	if (state == acs_ON)
	{
		HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_RESET);
	}
}

void AC_5kW(eACState state)
{
	if (state == acs_ON)
	{
		HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_RESET);
	}
}
