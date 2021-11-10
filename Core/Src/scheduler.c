// Simple scheduler for periodic tasks
// Author : Martin Kresta 
// Date   : 15.10.2019
// Project: Robodof


#include <SHUNT.h>
#include "UI.h"
#include "scheduler.h"
#include "OW.h"
#include "TEMP.h"
#include "SCOM.h"
#include "BMS1.h"
#include "BMS2.h"
#include "watchdog.h"
#include "AC.h"
#include "MCAN.h"
#include "COM.h"
#include "MPPT.h"
#include "ELECON.h"




/*************************
	V A R I A B L E 
*************************/
/** Timing struct for 1ms cycle*/
sTimer		Timer_1ms;
/** Timing struct for 5ms cycle*/
sTimer		Timer_5ms;
/** Timing struct for 10ms cycle*/
sTimer		Timer_10ms;
/** Timing struct for 50ms cycle*/
sTimer		Timer_50ms;
/** Timing struct for 100ms cycle*/
sTimer		Timer_100ms;
/** Timing struct for 250ms cycle*/
sTimer		Timer_250ms;
/** Timing struct for 500ms cycle*/
sTimer		Timer_500ms;
/** Timing struct for 1s cycle*/
sTimer		Timer_1s;

/*************************
	P R O T O T Y P E
*************************/
static void Timer_Increment (sTimer *timer);
static void Timer_Task_1ms(void);
static void Timer_Task_5ms(void);
static void Timer_Task_10ms(void);
static void Timer_Task_50ms(void);
static void Timer_Task_100ms(void);
static void Timer_Task_250ms(void);
static void Timer_Task_500ms(void);
static void Timer_Task_1s(void);

//static void Timer_Check_Flag(void);

/*************************
	F U N C T I O N
*************************/

/**
**********************************************************************	
	* @brief	Initialisation timimg struct
***********************************************************************
*/
void Scheduler_Init (void)
{
		Timer_1ms.timer_limit = 1;
		Timer_5ms.timer_limit = 5;
		Timer_10ms.timer_limit = 10;
		Timer_50ms.timer_limit = 50;
		Timer_100ms.timer_limit = 100;
		Timer_250ms.timer_limit = 250;
		Timer_500ms.timer_limit = 500;
		Timer_1s.timer_limit = 1000;
}

/**
**********************************************************************	
	* @brief	Timer control. Calling of all defined timers
***********************************************************************
*/
void Scheduler_Update_1ms (void)
{
	Timer_Increment(&Timer_1ms);
	Timer_Increment(&Timer_5ms);
	Timer_Increment(&Timer_10ms);
	Timer_Increment(&Timer_50ms);
	Timer_Increment(&Timer_100ms);
	Timer_Increment(&Timer_250ms);
	Timer_Increment(&Timer_500ms);
	Timer_Increment(&Timer_1s);
	
}
/**
**********************************************************************	
	* @brief	Incrementing of all counters and setting flags , where is condition met!
	* @param	timer: Pointer to specific timer	
***********************************************************************
*/
inline static void Timer_Increment (sTimer *timer)
{
	timer->timer_cnt++;
	if(timer->timer_cnt >= timer->timer_limit)
	{
		timer->flag = eFLAG_REACHED;
		timer->timer_cnt = 0;
	}
}

/**
**********************************************************************	
	* @brief Checking flags in structures and call specific tasks	
***********************************************************************
*/

void Scheduler_Check_Flag(void)
{
	if(Timer_1ms.flag)
	{
		Timer_Task_1ms();
		Timer_1ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_5ms.flag)
	{
		Timer_Task_5ms();
		Timer_5ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_10ms.flag)
	{
		Timer_Task_10ms();
		Timer_10ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_50ms.flag)
	{
		Timer_Task_50ms();
		Timer_50ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_100ms.flag)
	{
		Timer_Task_100ms();
		Timer_100ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_250ms.flag)
	{
		Timer_Task_250ms();
		Timer_250ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_500ms.flag)
	{
		Timer_Task_500ms();
		Timer_500ms.flag = eFLAG_NOT_YET;
	}
	
	if(Timer_1s.flag)
	{
		Timer_Task_1s();
		Timer_1s.flag = eFLAG_NOT_YET;
	}	
}

/**
**********************************************************************	
	* @brief 1ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_1ms(void)
{	
	//DI_Read_All();
}

/**
**********************************************************************	
	* @brief 1ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_5ms(void)
{	
	MCAN_Transmit();
}

/**
**********************************************************************	
	* @brief 10ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_10ms(void)
{
	MCAN_Update_10ms();
	COM_Update_10ms();
	UI_Update_10ms();
	SCOM_Update_10ms();
}
/**
**********************************************************************
	* @brief 10ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_50ms(void)
{
	//LC_Update_50ms();
}

/**
**********************************************************************	
	* @brief 100ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_100ms(void)
{

	TEMP_Update100ms();
	MPPT_Update_100ms();
	SHUNT_Update_100ms();
//	ADC_StartConversion();
}

/**
**********************************************************************	
	* @brief 250ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_250ms(void)
{
}
	
/**
**********************************************************************	
	* @brief 500ms Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_500ms(void)
{
	//HAL_GPIO_TogglePin(LED_B_GPIO_Port,LED_B_Pin);
	BMS1_Update_500ms();
	BMS2_Update_500ms();

}

/**
**********************************************************************	
	* @brief 1s Task ( independent on interrupt)
	* @param
	* @retval
***********************************************************************
*/
static void Timer_Task_1s(void)
{	
	WDG_Refresh();
	AC_Update_1s();
	ELC_Update_1s();
	APP_Update_1s();
	//LED_Error_SetMode(eLED_BLINK_ONCE);
	//	OW_Read(0);
	//		OW_ConvertAll(0);
	//OW_ReadRom(0);
	//OW_Read(0);
	//OW_ConvertAll(0);


}
