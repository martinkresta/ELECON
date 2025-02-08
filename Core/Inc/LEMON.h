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
  float OptChargingCurrent;
}sStrg;   // published  values

typedef struct
{
  uint64_t Avilable_mAs;
  float AvgBmsVoltage;
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
  uint16_t Pack1_Ah;
  uint16_t Pack2_Ah;
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
  sBMS* Bms1;
  sBMS* Bms2;
}sLemon;




void LEMON_Init(UART_HandleTypeDef* huart1, UART_HandleTypeDef* huart2);
void LEMON_Update_1s(void);
void LEMON_MidnightNow(void);
void LEMON_UartRxCallback(USART_TypeDef uart, uint16_t reclength);



#endif /* INC_LEMON_H_ */
