/*
 * GEMON.c
 *
 *  Created on: 8. 2. 2025
 *      Author: marti
 *      Brief: Global Energy MONitoring
 *             SW module for combining data from 2 LEMONs :-)
 *             - Uses only data from VARs, so it is independent on hardware.
 */

#ifndef INC_GEMON_H_
#define INC_GEMON_H_


#include "main.h"
#include "VARS.h"
#include "RTC.h"

typedef struct
{
  float SolarPower;
  float SolarProd_Wh;
  float LoadPower;
  float LoadCons_Wh;
  float BatSoc;
  float BatPower;
  float BatVoltage;
  float BatCurrent;
  float BatEnergy;
}sGemon;

typedef struct
{
  uint8_t BalancedToday;
  float OptChargeCurrent;
}sStrgUtilInfo;


typedef struct
{
  float OptChargeCurrent;
  int64_t RemainingTime_s;
  uint8_t ChargeTargetHour;
  sDateTime now;
  sStrgUtilInfo Strg1;
  sStrgUtilInfo Strg2;
}sElUtil;



void GEMON_Init(void);
void GEMON_Update_1s(void);


#endif /* INC_GEMON_H_ */
