// Simple scheduler for periodic tasks
// Author : Vaclav Silar, Martin Kresta 
// Date   : 15.10.2019
// Project: Robodof
// Brief  : Tasks has same priority, never preempted, jitter may occur


#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "main.h"


/*************************
	T Y P E D E F
*************************/
typedef enum
{
	eFLAG_NOT_YET = 0,
	eFLAG_REACHED	
}eTimer_Flag;

typedef struct
{
	eTimer_Flag		flag;
	uint16_t			timer_cnt;
	uint16_t			timer_limit;
}sTimer;

/*************************
	P R O T O T Y P E
*************************/
void Scheduler_Init (void);
void Scheduler_Update_1ms (void);
void Scheduler_Check_Flag(void);




#endif  // SCHEDULER_H


