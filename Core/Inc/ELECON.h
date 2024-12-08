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
#define PACK5KWH_EFF_CAPACITY_AH     80
#define PACK14KWH_EFF_CAPACITY_AH     220
#define AH2MAS									3600000

#define CELL_BALANCE_MV					3400
#define CELL_MAX_MV							3500
#define CELL_TARGET_MV					3450

#define BAT_FULL_TARGET_HOUR			15

typedef struct   // data of one battery pack monitored by shunt coulombmeter
{
  int32_t Soc_pct100;
  int32_t Energy_Wh;
  int32_t Power_W;
  int32_t Voltage_V10;        // BMS
  int64_t Available_mAs;
  int32_t Current_mA;
  uint8_t ChargingEnabled;    // BMS
  uint8_t DischargingEnabled;
}sPackInfo;


typedef struct   // agregated data of all battery Packs - overal storage capacity and state
{
  int32_t Soc_pct100;
  int32_t Energy_Wh;
  int32_t Power_W;
  int32_t Voltage_V10;
  int64_t Available_mAs;
  int32_t Current_mA;
}sStorageInfo;

void ELC_Init(void);

void ELC_Update_1s(void);

void ELC_MidnightNow(void);



#endif /* INC_ELECON_H_ */
