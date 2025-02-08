/*
 * BCKPBAT.c
 *
 *  Created on: 8. 2. 2025
 *      Author: marti
 *      Brief: Monitoring and charging control of Auxiliary backup 12V PB battery (Used for emergency when main storage is shut down)
 */


#include "main.h"
#include "ADC.h"
#include "UI.h"

void BCKPBAT_Update_1s(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint16_t VbatRaw;
  //uint16_t IbatRaw;
  uint8_t BackupOn = 0;
  BackupOn = HAL_GPIO_ReadPin(BCKP_STATE_GPIO_Port, BCKP_STATE_Pin);
  // Read Aux battery voltage

  VbatRaw = ADC_GetValue(ADC_CHANNEL_AUX_BAT_V);  // raw ADC result
  double Vbat_mV = (ADC_VREF_MV/4096.0 * VbatRaw * 12) / 2.44 ;  // convert to milivolts
  // Read Aux battery current
  //IbatRaw = ADC_GetValue(ADC_CHANNEL_AUX_BAT_I);  // raw ADC result
  //double Ibat_mA = (ADC_VREF_MV/4096.0 * IbatRaw) * 2.128 ;  // convert to miliamperes

  // check status of backup
  if (BackupOn == 1)
  {
    UI_LED_B_SetMode(eUI_BLINKING_FAST);   // signalization of main battery powerdown
  }
  else
  {
    UI_LED_B_SetMode(eUI_OFF);
  }

  // enable/disable charging
  if(Vbat_mV > 13900)
  {
    HAL_GPIO_WritePin(CHARGE_ENA_GPIO_Port, CHARGE_ENA_Pin, GPIO_PIN_RESET);  // disable charging
  }
  else if (Vbat_mV < 13000)
  {
    HAL_GPIO_WritePin(CHARGE_ENA_GPIO_Port, CHARGE_ENA_Pin, GPIO_PIN_SET);  // enable charging
  }

  // enable disable backup
  if(Vbat_mV > 12000)
  {
    GPIO_InitStruct.Pin = BCKP_ENA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(BCKP_ENA_GPIO_Port, BCKP_ENA_Pin, GPIO_PIN_RESET);  // enable backup
  }
  else if (Vbat_mV < 11000)
  {
     /*Configure GPIO pins : BCKP_ENA_Pin */
    GPIO_InitStruct.Pin = BCKP_ENA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;       // HiZ = disable backup
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  }
}
