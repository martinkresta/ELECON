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


#define AC_MIN_ON_TIME_S			180
#define AC_KEEP_ON_POWER_W_5KW		15   // 4 minutes
#define AC_KEEP_ON_POWER_W_3KW		6
#define AC_KEEP_ON_POWER_W_300W    20   // 3 minutes

typedef enum
{
	acs_OFF = 0,
	acs_ON,
	acs_TOGGLE
}
eACControl;


typedef enum
{
  acs_Off,
  acs_On,
  acs_OnKeepSetting,
  acs_KeepOn,
  acs_AutoOff
}eACState;


typedef struct
{
  eACState state;
  uint32_t OnTimer;
  uint32_t SettingTimer;
  uint8_t EnableAutoOff;
  uint32_t MinOnTime;
  uint8_t AutoOfPower;
  uint8_t ElmeterId;
  uint8_t RemoteRequests;
  GPIO_TypeDef* RelayPort;
  uint16_t RelayPin;
}sDCAC;

typedef enum
{
  eDCAC_300W = 0,
  eDCAC_3kW,
  eDCAC_5kW,
}eDCACType;



void AC_Init(void);

void AC_Update_1s(void);

void AC_3kW_ButtonGesture(void);
void AC_5kW_ButtonGesture(void);

void AC_RemoteRequest(eDCACType type, eACControl request, uint32_t keepOnTime);

/*
void AC_300W(eACControl state);

void AC_3kW(eACControl state);

void AC_5kW(eACControl state);
*/


#endif /* INC_AC_H_ */
