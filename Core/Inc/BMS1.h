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

__packed typedef struct{
	uint16_t Voltage_mV;
	int16_t Temp_C;
}sCell;

__packed typedef struct{
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


void BMS1_UartRxCallback(void);



#endif
