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
#include "VARS2.h"

uint16_t m5KwStateTimer_s;   // time elapsed since laste state change
uint16_t m3KwStateTimer_s;   // time elapsed since laste state change
uint8_t mEnableAutoOff;


sDCAC AC300W;
sDCAC AC3kW;
sDCAC AC5kW;



void SwitchRelay(eACControl state,GPIO_TypeDef* port, uint16_t pin);
void AcStateMachine(sDCAC* AC);
void ButtonGesture(sDCAC* AC);
void RemoteRequest(sDCAC* AC, eACControl request, uint16_t keepOnTime);

static const sVarsMetersIoboardD* mpKitchen;   /* read: kitchen power (IOBOARD_D) */

void AC_Init(const sVarsMetersIoboardD* pKitchen)
{
  mpKitchen = pKitchen;
  AC300W.AutoOfPower = AC_KEEP_ON_POWER_W_300W;
  AC300W.EnableAutoOff = 0;
  AC300W.MinOnTime = 0;
  AC300W.state = acs_On;
  AC300W.OnTimer = 0;
  AC300W.ElmeterId = ELM_AC_300W;
  AC300W.RemoteRequests = 0;
  AC300W.RelayPin = REL1_Pin;
  AC300W.RelayPort = REL1_GPIO_Port;
  SwitchRelay(acs_ON, AC300W.RelayPort, AC300W.RelayPin);

  AC3kW.AutoOfPower = AC_KEEP_ON_POWER_W_3KW;
  AC3kW.EnableAutoOff = 1;
  AC3kW.MinOnTime = 15*60;  // 15 min
  AC3kW.state = acs_Off;
  AC3kW.OnTimer = 0;
  AC3kW.ElmeterId = ELM_AC_3KW;
  AC3kW.RemoteRequests = 0;
  AC3kW.RelayPin = REL2_Pin;
  AC3kW.RelayPort = REL2_GPIO_Port;
  SwitchRelay(acs_OFF, AC3kW.RelayPort, AC3kW.RelayPin);

  AC5kW.AutoOfPower = AC_KEEP_ON_POWER_W_5KW;
  AC5kW.EnableAutoOff = 1;
  AC5kW.MinOnTime = 5*60;  // 5min
  AC5kW.state = acs_Off;
  AC5kW.OnTimer = 0;
  AC5kW.ElmeterId = ELM_AC_5KW;
  AC5kW.RemoteRequests = 0;
  AC5kW.RelayPin = REL3_Pin;
  AC5kW.RelayPort = REL3_GPIO_Port;
  SwitchRelay(acs_OFF, AC5kW.RelayPort, AC5kW.RelayPin);


}

void AC_Update_1s(void)
{
	sDateTime now = RTC_GetTime();

  // Configure 300 auto off conditions w.r.t DARK_SEASON

  if ( now.Month <= 2 || now.Month >= 11)  // DARK_SEASON from November to February
  {
    AC300W.EnableAutoOff = 1;
  }
  else  // outside the DARK_SEASON we do not turn off the 300W DCAC
  {
    AC300W.EnableAutoOff = 0;
  }

	// start the 300W ac every FRIDGE_PERIOD  (every hour)   (applicable only in dark season)
	if(now.Minute == 0 && now.Second < 2  && AC300W.state == acs_Off)
	{
	  if(AC300W.OnTimer < 300)
    {
	    AC300W.OnTimer = 300;  // during the 5 minutes the fridge should start and than keep the DCAC by the load
    }
	  AC300W.state = acs_KeepOn;
    SwitchRelay(acs_ON, AC300W.RelayPort, AC300W.RelayPin);
	}

	// start the 300W AC when kitchen consumption is significant -> enable air cleaner   (applicable only in dark season)
	if((50.0f < mpKitchen->KitchenPower_W.value) && AC300W.state == acs_Off)   // validity not checked, as before
	{
	  if(AC300W.OnTimer < 180)
    {
      AC300W.OnTimer = 180;  // during the 3 minutes the air cleaner should start and than keep the DCAC by the load
    }
    AC300W.state = acs_KeepOn;
    SwitchRelay(acs_ON, AC300W.RelayPort, AC300W.RelayPin);
	}




	AcStateMachine(&AC300W);
	AcStateMachine(&AC3kW);
	AcStateMachine(&AC5kW);
}


void AcStateMachine(sDCAC* AC)
{

  switch(AC->state)
  {
    case acs_Off:
      break;
    case acs_On:
      if (AC->EnableAutoOff)
      {
        AC->state = acs_KeepOn;
      }
      break;
    case acs_OnKeepSetting:
      AC->SettingTimer --;
      if (AC->SettingTimer == 0)
      {
        AC->state = acs_KeepOn;
      }
      break;
    case acs_KeepOn:
      if(AC->OnTimer > 0)
      {
        AC->OnTimer --;
      }
      if (AC->OnTimer == 0)
      {
        AC->state = acs_AutoOff;
      }
      break;
    case acs_AutoOff:
      if (AC->EnableAutoOff)
      {
        if (ELM_GetPowerW(AC->ElmeterId) < AC->AutoOfPower)
        {
          SwitchRelay(acs_OFF, AC->RelayPort, AC->RelayPin);
          AC->state = acs_Off;
        }
      }
      else
      {
        AC->state = acs_On;
      }
      break;
  }
}



// reaction to detected button gesture
void ButtonGesture(sDCAC* AC)
{
  switch(AC->state)
  {
    case acs_Off:
      AC->OnTimer = AC->MinOnTime;
      AC->state = acs_OnKeepSetting;
      AC->SettingTimer = 5;
      SwitchRelay(acs_ON, AC->RelayPort, AC->RelayPin);
      break;
    case acs_On:
      SwitchRelay(acs_OFF, AC->RelayPort, AC->RelayPin);
      AC->state = acs_Off;
      break;
    case acs_OnKeepSetting:
     //->SettingTimer = 5;  // refresh setting timeout

      AC->OnTimer += 60*60*2;  // 2 hours
      break;
    case acs_KeepOn:
      SwitchRelay(acs_OFF, AC->RelayPort, AC->RelayPin);
      AC->state = acs_Off;
      break;
    case acs_AutoOff:
      SwitchRelay(acs_OFF, AC->RelayPort, AC->RelayPin);
      AC->state = acs_Off;
      break;
  }
}



void SwitchRelay(eACControl state,GPIO_TypeDef* port, uint16_t pin)
{
  if (state == acs_ON)
  {
    HAL_GPIO_WritePin(port,pin,GPIO_PIN_SET);
  }
  else if (state == acs_OFF)
  {
    HAL_GPIO_WritePin(port,pin,GPIO_PIN_RESET);
  }
  else if (state == acs_TOGGLE)
  {
    HAL_GPIO_TogglePin(port, pin);
  }
}



void AC_3kW_ButtonGesture(void)
{
  ButtonGesture(&AC3kW);
}


void AC_5kW_ButtonGesture(void)
{
  ButtonGesture(&AC5kW);
}

// command channel (APP_OnCommand): on = the same as a person at the switch, only from acs_Off (ButtonGesture():
// min on time, then auto off by power); already on -> ignored (no +2 h, no timer restart).
// off = the off branch of ButtonGesture() (relay off, acs_Off) in every on state, also in acs_OnKeepSetting
// (where a gesture would add 2 h); already off -> ignored. RemoteRequests (TECHM, 0x130) are not touched.
static void Command(sDCAC* AC, uint8_t on)
{
  if (on)
  {
    if (AC->state == acs_Off)
    {
      ButtonGesture(AC);
    }
  }
  else if (AC->state != acs_Off)
  {
    SwitchRelay(acs_OFF, AC->RelayPort, AC->RelayPin);
    AC->state = acs_Off;
  }
}

// 3 kW / 5 kW only; the 300 W has no remote on / off (always on, its own auto start / auto off)
void AC_Command(eDCACType type, uint8_t on)
{
  switch (type)
  {
    case eDCAC_3kW:
      Command(&AC3kW, on);
      break;
    case eDCAC_5kW:
      Command(&AC5kW, on);
      break;
    default:
      break;
  }
}

void AC_RemoteRequest(eDCACType type, eACControl request, uint16_t keepOnTime)
{
  switch (type)
  {
    case eDCAC_300W:
      RemoteRequest(&AC300W, request, keepOnTime);
      break;
    case eDCAC_3kW:
      RemoteRequest(&AC3kW, request, keepOnTime);
      break;
    case eDCAC_5kW:
      RemoteRequest(&AC5kW, request, keepOnTime);
      break;
  }
}


void RemoteRequest(sDCAC* AC, eACControl request, uint16_t keepOnTime)
{
  if(request == acs_ON)
  {
    AC->RemoteRequests ++;
    if(AC->OnTimer < keepOnTime)
    {
      AC->OnTimer = keepOnTime;
    }
    AC->state = acs_KeepOn;
    SwitchRelay(acs_ON, AC->RelayPort, AC->RelayPin);
  }
  else if(request == acs_OFF)
  {
   // AC->OnTimer = 0;   // not necessary
    if(AC->RemoteRequests > 0)
    {
      AC->RemoteRequests --;
    }
    if(AC->RemoteRequests == 0)
    {
      AC->OnTimer = 0;
      AC->state = acs_AutoOff;
    }
  }
}

/*
// Obsolete functions :

void AC_300W(eACControl state)
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

void AC_3kW(eACControl state)
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

void AC_5kW(eACControl state)
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

*/

/* state of the AC state machine (read only, for the VARS2 states) */
eACState AC_GetState(eDCACType type)
{
  switch (type)
  {
    case eDCAC_300W:
      return AC300W.state;
    case eDCAC_3kW:
      return AC3kW.state;
    case eDCAC_5kW:
      return AC5kW.state;
  }
  return acs_Off;
}
