/*
 * ELECON.h
 *
 *  Created on: Nov 9, 2021
 *      Author: Martin
 */

#ifndef INC_ELECON_H_
#define INC_ELECON_H_

#include "main.h"


#define BAT_EFF_CAPACITY_AH			300
#define AH2MAS									3600000

#define CELL_BALANCE_MV					3400
#define CELL_MAX_MV							3500
#define CELL_TARGET_MV					3450


void ELC_Init(void);

void ELC_Update_1s(void);

void ELC_MidnightNow(void);



#endif /* INC_ELECON_H_ */
