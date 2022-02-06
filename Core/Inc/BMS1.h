/*
 * BMS1.h
 *
 *  Created on: Oct 24, 2021
 *      Author: Martin
 *       Brief: Module for one-way communication with BMS on battery pack ()
 */


#ifndef INC_BMS1_H_
#define INC_BMS1_H_


#include "main.h"

#define BMS_DATA_LENGTH 58   // documantation says 58,
#define DMA_REC_LENGTH 59 // maximal allowed length of DMA transfer


#define BMS_STAT_ALLOW_CHARGE					0x01
#define BMS_STAT_ALLOW_DISCHARGE			0x02
#define BMS_STAT_COM_ERROR						0x04
#define BMS_STAT_CELL_LOW_VOLTAGE			0x08
#define BMS_STAT_CELL_HIGH_VOLTAGE		0x10
#define BMS_STAT_LOW_TEMPERATURE			0x20
#define BMS_STAT_HIGH_TEMPERATURE			0x40
#define BMS_STAT_SOC_UNKNOWN					0x80

typedef struct{
	uint16_t Voltage_mV;
	int16_t Temp_C;
}sCell;

typedef struct{
	uint32_t Capacity_Wh;
	uint32_t Vmin_mV;
	uint32_t Vmax_mV;
	uint32_t Vbalance_mV;
	uint8_t NumOfCells;
}sPackInfo;

typedef struct{
	uint32_t VoltageTotal_mV;
	int32_t ChargingCurrent_mA;
	int32_t DischargingCurrent_mA;
	int32_t BatteryCurrent_mA;
	uint32_t TodayCharging_Wh;
	uint32_t TodayDischarging_Wh;
	uint32_t Energystored_Wh;
	uint32_t TotalCharging_kWh;
	uint32_t TotalDischarging_kWh;
	uint8_t Status;
	uint8_t SOC;
	//sCell* Cells;   // dynamically allocated array of cell modules
}sLiveData;


void BMS1_Init(UART_HandleTypeDef* huart);
// Update function, to the called periodically by the scheduler
void BMS1_Update_500ms(void);
// Gets maximal cell voltage
uint16_t BMS1_GetMaxCellVoltage(void);
// Gets minimal cell voltage
uint16_t BMS1_GetMinCellVoltage(void);
// gets status of charge enable relay
uint8_t BMS1_IsChargingEnabled(void);


void BMS1_UartRxCallback(uint16_t reclength);



#endif
