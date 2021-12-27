/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern CAN_HandleTypeDef hcan1;

extern IWDG_HandleTypeDef hiwdg;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern RTC_HandleTypeDef hrtc;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim2;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IO1_Pin GPIO_PIN_13
#define IO1_GPIO_Port GPIOC
#define BCKP_STATE_Pin GPIO_PIN_14
#define BCKP_STATE_GPIO_Port GPIOC
#define BCKP_ENA_Pin GPIO_PIN_15
#define BCKP_ENA_GPIO_Port GPIOC
#define MPPT_RX_Pin GPIO_PIN_0
#define MPPT_RX_GPIO_Port GPIOC
#define MPPT_TX_Pin GPIO_PIN_1
#define MPPT_TX_GPIO_Port GPIOC
#define EL1_Pin GPIO_PIN_2
#define EL1_GPIO_Port GPIOC
#define EL1_EXTI_IRQn EXTI2_IRQn
#define EL2_Pin GPIO_PIN_3
#define EL2_GPIO_Port GPIOC
#define EL2_EXTI_IRQn EXTI3_IRQn
#define EL3_Pin GPIO_PIN_0
#define EL3_GPIO_Port GPIOA
#define EL3_EXTI_IRQn EXTI0_IRQn
#define BAT_VOLTAGE_Pin GPIO_PIN_1
#define BAT_VOLTAGE_GPIO_Port GPIOA
#define IO4_Pin GPIO_PIN_3
#define IO4_GPIO_Port GPIOA
#define OUT4_S_Pin GPIO_PIN_4
#define OUT4_S_GPIO_Port GPIOA
#define OUT3_S_Pin GPIO_PIN_5
#define OUT3_S_GPIO_Port GPIOA
#define OUT2_S_Pin GPIO_PIN_6
#define OUT2_S_GPIO_Port GPIOA
#define OUT1_S_Pin GPIO_PIN_7
#define OUT1_S_GPIO_Port GPIOA
#define LATCH_REL_OFF_Pin GPIO_PIN_5
#define LATCH_REL_OFF_GPIO_Port GPIOC
#define LATCH_REL_ON_Pin GPIO_PIN_0
#define LATCH_REL_ON_GPIO_Port GPIOB
#define BAT_CURR_Pin GPIO_PIN_1
#define BAT_CURR_GPIO_Port GPIOB
#define OUT4_Pin GPIO_PIN_2
#define OUT4_GPIO_Port GPIOB
#define OUT3_Pin GPIO_PIN_10
#define OUT3_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_11
#define OUT2_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_12
#define OUT1_GPIO_Port GPIOB
#define REL3_Pin GPIO_PIN_13
#define REL3_GPIO_Port GPIOB
#define REL2_Pin GPIO_PIN_14
#define REL2_GPIO_Port GPIOB
#define REL1_Pin GPIO_PIN_15
#define REL1_GPIO_Port GPIOB
#define OW1_Pin GPIO_PIN_6
#define OW1_GPIO_Port GPIOC
#define LED_Life_Pin GPIO_PIN_7
#define LED_Life_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_8
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_9
#define LED_G_GPIO_Port GPIOC
#define LED_B_Pin GPIO_PIN_8
#define LED_B_GPIO_Port GPIOA
#define RPI_TX_Pin GPIO_PIN_9
#define RPI_TX_GPIO_Port GPIOA
#define RPI_RX_Pin GPIO_PIN_10
#define RPI_RX_GPIO_Port GPIOA
#define BMS2_RX_Pin GPIO_PIN_15
#define BMS2_RX_GPIO_Port GPIOA
#define SHUNT1_CS_Pin GPIO_PIN_10
#define SHUNT1_CS_GPIO_Port GPIOC
#define BMS1_RX_Pin GPIO_PIN_11
#define BMS1_RX_GPIO_Port GPIOC
#define SHUNT1_RST_Pin GPIO_PIN_12
#define SHUNT1_RST_GPIO_Port GPIOC
#define SHUNT2_CS_Pin GPIO_PIN_2
#define SHUNT2_CS_GPIO_Port GPIOD
#define SHUNT2_RST_Pin GPIO_PIN_6
#define SHUNT2_RST_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_7
#define BUZZ_GPIO_Port GPIOB
#define CHARGE_ENA_Pin GPIO_PIN_3
#define CHARGE_ENA_GPIO_Port GPIOH
#define IO3_Pin GPIO_PIN_8
#define IO3_GPIO_Port GPIOB
#define IO2_Pin GPIO_PIN_9
#define IO2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
