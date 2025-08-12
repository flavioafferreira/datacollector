/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins
*/
void MX_GPIO_Init(void)
{

  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(alim_ntc_GPIO_Port, alim_ntc_Pin);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_12;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

  /**/
  LL_GPIO_SetPinMode(memory_print_GPIO_Port, memory_print_Pin, LL_GPIO_MODE_INPUT);

  /**/
  LL_GPIO_SetPinPull(memory_print_GPIO_Port, memory_print_Pin, LL_GPIO_PULL_DOWN);

  /**/
  LL_EXTI_SetEXTISource(LL_EXTI_CONFIG_PORTA, LL_EXTI_CONFIG_LINE12);

  /**/
  GPIO_InitStruct.Pin = alim_ntc_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(alim_ntc_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  NVIC_SetPriority(EXTI4_15_IRQn, 0);
  NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 2 */
void Pin_Config_NTC_Alim(void){
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	/** Configure alim_ntc as output BEFORE setting its value */
	  GPIO_InitStruct.Pin = alim_ntc_Pin;
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	  LL_GPIO_Init(alim_ntc_GPIO_Port, &GPIO_InitStruct);
	  /* Now it's safe to set it high */
	  LL_GPIO_SetOutputPin(alim_ntc_GPIO_Port, alim_ntc_Pin);
}

void Pin_Config_NTC_Disable(void){
	   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	    GPIO_InitStruct.Pin = alim_ntc_Pin;
	    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN; // ou LL_GPIO_PULL_UP se preferir
	    LL_GPIO_Init(alim_ntc_GPIO_Port, &GPIO_InitStruct);
}



/* USER CODE END 2 */
