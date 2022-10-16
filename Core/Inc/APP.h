/*
 * APP.h
 *
 *  Created on: Aug 14, 2021
 *      Author: Martin
*				Brief:  Main application and compile time configuration
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "main.h"
#include "MCAN.h"
#include "stdbool.h"


// CAN node ID
#define THIS_NODE				NODEID_ELECON


#define NUM_OF_ELEMTERS			3
#define ELM_AC_300W					0
#define ELM_AC_3KW					1
#define ELM_AC_5KW					2


void APP_Init(void);
void APP_Start(void);
void APP_ProcessMessages(void);
void APP_Update_1s(void);


#endif /* INC_APP_H_ */
