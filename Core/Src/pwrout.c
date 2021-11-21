/*
 * pwrout.c - module for control of power outputs with PWM capability.
 * 					- based on timer OC channels
 * 					- Warning ! Only OUTPUTS OUT2 and OUT3 have PWM capability. OUT1 and OUT4 are just ON/OFF
 *
 *  Created on: Apr 29, 2021
 *      Author: Martin
 */


#include "pwrout.h"
#include "main.h"

static sPwrOut Outputs[NUM_OF_OUTPUTS];

static TIM_OC_InitTypeDef sConfigOC;


static void 		PWROUT_Struct_Initialisation (sPwrOut init_struct, uint8_t output);

void PWROUT_Init(void)
{
	// prepare structure for configuring OC channels
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	sPwrOut init_struct;

	init_struct.DutyCycle = 0;
	init_struct.Timer = &htim2;
	init_struct.Channel = TIM_CHANNEL_4;
	PWROUT_Struct_Initialisation(init_struct, OUT2);

	init_struct.Channel = TIM_CHANNEL_3;
	PWROUT_Struct_Initialisation(init_struct, OUT3);

	// start all used timers in PWM mode
}

void PWROUT_SetPct(uint8_t output,uint8_t dutyCycle)
{
	if (output == OUT2 || output == OUT3)  // PWM outputs
	{
		sConfigOC.Pulse = dutyCycle * 10;
		Outputs[output].DutyCycle = dutyCycle;

		HAL_TIM_PWM_Stop(Outputs[output].Timer, Outputs[output].Channel); // we have to stop here

		if (HAL_TIM_PWM_ConfigChannel(Outputs[output].Timer, &sConfigOC, Outputs[output].Channel) != HAL_OK)  // because this fcn will stop it in register CCER - CCxE
		{
			Error_Handler();
		}
		HAL_TIM_PWM_Start(Outputs[output].Timer, Outputs[output].Channel);  // and this fcn is then not able to start  (HAL HELL ! )
	}
	else  // on off outputs
 	{
		Outputs[output].DutyCycle = dutyCycle;
		if (output == OUT1)
		{
			if(dutyCycle)
			{
				HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);
			}
		}
		else if(output == OUT4)
		{
			if(dutyCycle)
			{
				HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET);
			}
		}
	}


}

uint8_t PWROUT_GetPct(uint8_t output)
{
	return Outputs[output].DutyCycle;
}


static void 		PWROUT_Struct_Initialisation (sPwrOut init_struct, uint8_t output)
{
	Outputs[output].Timer = init_struct.Timer;
	Outputs[output].Channel = init_struct.Channel;
	Outputs[output].DutyCycle = init_struct.DutyCycle;


	HAL_TIM_PWM_Start(init_struct.Timer, init_struct.Channel);
}
