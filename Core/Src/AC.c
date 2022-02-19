/*
 * REL.c
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 */


#include "AC.h"
#include "RTC.h"
#include "ELM.h"
#include "APP.h"

uint16_t m5KwStateTimer_s;   // time elapsed since laste state change
uint16_t m3KwStateTimer_s;   // time elapsed since laste state change
uint8_t mEnableAutoOff;


void AC_Init(void)
{
	AC_300W(acs_ON);
	AC_3kW(acs_OFF);
	AC_5kW(acs_OFF);
	mEnableAutoOff = 1;
}

void AC_Update_1s(void)
{

	sDateTime now = RTC_GetTime();
	if(now.Hour == 5  && now.Minute == 0 && now.Second == 0)
	{
	//	AC_3kW(acs_ON);
	//	AC_5kW(acs_ON);
	}
	else if (now.Hour == 22  && now.Minute == 0 && now.Second == 0)
	{
		AC_3kW(acs_OFF);
		AC_5kW(acs_OFF);
	}

	m3KwStateTimer_s ++;
	m5KwStateTimer_s ++;

	// auto off feature for 5kW and 3kW DC/AC
  if (mEnableAutoOff)
  {
  	if (m3KwStateTimer_s  > AC_MIN_ON_TIME_S)
  	{
  		if (ELM_GetPowerW(ELM_AC_3KW) < AC_KEEP_ON_POWER_W_3KW)
  		{
  			AC_3kW(acs_OFF);
  		}
  	}

		if (m5KwStateTimer_s  > AC_MIN_ON_TIME_S)
		{
			if (ELM_GetPowerW(ELM_AC_5KW) < AC_KEEP_ON_POWER_W_5KW)
			{
				AC_5kW(acs_OFF);
			}
		}
  }
}

void AC_300W(eACState state)
{
	if (state == acs_ON)
	{
		HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_SET);
	}
	else if (state == acs_OFF)
	{
		HAL_GPIO_WritePin(REL1_GPIO_Port,REL1_Pin,GPIO_PIN_RESET);
	}
	else if (state == acs_TOGGLE)
	{
		HAL_GPIO_TogglePin(REL1_GPIO_Port, REL1_Pin);
	}
}

void AC_3kW(eACState state)
{
	if (state == acs_ON)
	{
		HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_SET);
	}
	else if (state == acs_OFF)
	{
		HAL_GPIO_WritePin(REL2_GPIO_Port,REL2_Pin,GPIO_PIN_RESET);
	}
	else if (state == acs_TOGGLE)
	{
		HAL_GPIO_TogglePin(REL2_GPIO_Port, REL2_Pin);
	}
	m3KwStateTimer_s = 0;
}

void AC_5kW(eACState state)
{
	if (state == acs_ON)
	{
		HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_SET);
	}
	else if (state == acs_OFF)
	{
		HAL_GPIO_WritePin(REL3_GPIO_Port,REL3_Pin,GPIO_PIN_RESET);
	}
	else if (state == acs_TOGGLE)
	{
		HAL_GPIO_TogglePin(REL3_GPIO_Port, REL3_Pin);
	}
	m5KwStateTimer_s = 0;
}
