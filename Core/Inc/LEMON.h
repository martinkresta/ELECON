/*
 * LEMON.h
 *
 *  Created on: 7. 2. 2025
 *      Author: marti
 *      Brief: Local Energy MONitoring
 *             SW module for monitoring local Storage, PV (production) and Load (consumption)
 *             - Uses Drivers for accessing BMS, Shunt, MPPT and ACDC inverter
 */


#ifndef INC_LEMON_H_
#define INC_LEMON_H_

#include "BMS.h"
#include "uha_vars_gen.h"   /* Vars2 group types */


#define AH2MAS                  3600000

#define CELL_BALANCE_MV         3400
#define CELL_MAX_MV             3500
#define CELL_TARGET_MV          3450

#define BAT_FULL_TARGET_HOUR      15


typedef struct
{
  float Soc;
  float Power;
  float Current;
  float Energy;
  float Voltage;
  float OptChargingCurrent;
}sStrg;   // published  values

typedef struct
{
  uint64_t Avilable_mAs;
  //float AvgBmsVoltage;
  float TotalCapacity_Ah;
  uint16_t MaxCellVoltage_mV;
  uint16_t MinCellVoltage_mV;
}sStrgPrivate;   // private values

typedef struct
{
  float Power;
  float Cons_Wh;
  float LoadCurrent;
}sLoad;


typedef struct
{
  float Power;
  float Voltage;
  float Current;
}sStringData;

typedef struct
{
  float Power;
  float Prod_Wh;
  float ChargeCurrent;
  sStringData String1;
  sStringData String2;
}sSolar;

typedef struct
{
  uint16_t Pack80_Ah;    // house pack 80 Ah, BMS on UART3
  uint16_t Pack220_Ah;   // house pack 220 Ah, BMS on UART2
  uint16_t Shunt_R_uOhm;
}
cLemonCfg;

typedef struct
{
  cLemonCfg Cfg;
  sStrg Storage;
  sLoad Load;
  sSolar Prod;
  uint8_t BalancedTodayFlag;
  uint8_t ChargingDisabledFlag;
  uint8_t SocInitialized;
  sStrgPrivate Internal;
  sBMS* Bms80;
  sBMS* Bms220;
}sLemon;




/* huart80 / huart220: BMS of the 80 Ah / 220 Ah pack
 * produced: pEnergy (Vars2.HomeEnergy), pPack80 / pPack220 (Vars2.PackHome80Ah / PackHome220Ah, published by
 *           the BMS driver); read: pMppt (Vars2.Mppt, solar power and charging current) */
void LEMON_Init(UART_HandleTypeDef* huart80, UART_HandleTypeDef* huart220, sVarsLocalEnergy* pEnergy,
                sVarsPack* pPack80, sVarsPack* pPack220, const sVarsMppt* pMppt);
void LEMON_Update_1s(void);
void LEMON_MidnightNow(void);
void LEMON_UartRxCallback(UART_HandleTypeDef *huart, uint16_t reclength);



#endif /* INC_LEMON_H_ */
