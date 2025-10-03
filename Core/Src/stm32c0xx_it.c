/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32c0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32c0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
extern uint16_t ad_value;
extern uint8_t calibration_completed;
extern uint32_t ext_cnt;
extern uint8_t print_memory;
extern uint8_t it_ready;


extern relay_st relay;
extern uint8_t triac_zc_on;
uint32_t impulse=0;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern flag_t Flg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32C0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32c0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC interrupts through EXTI lines 19 and 21.
  */
void RTC_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_IRQn 0 */
	if (LL_RTC_IsActiveFlag_ALRA(RTC)) {
	        LL_RTC_ClearFlag_ALRA(RTC);
	        LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_19);

	        // Acordou do modo STOP
	        // Aqui você pode acender um LED, setar uma flag, etc.
	}
  /* USER CODE END RTC_IRQn 0 */
  /* USER CODE BEGIN RTC_IRQn 1 */

  /* USER CODE END RTC_IRQn 1 */
}

/**
  * @brief This function handles Flash global interrupt.
  */
void FLASH_IRQHandler(void)
{
  /* USER CODE BEGIN FLASH_IRQn 0 */
 if (!calibration_completed)
  /* USER CODE END FLASH_IRQn 0 */
  HAL_FLASH_IRQHandler();
  /* USER CODE BEGIN FLASH_IRQn 1 */

  /* USER CODE END FLASH_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
  if(it_ready==ON)
  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_12) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_12);
    /* USER CODE BEGIN LL_EXTI_LINE_12_RISING */
    if (it_ready == ON){
     LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_12); // evita bounce/reentrância
     print_memory=ON;
     it_ready=OFF;
    }
    /* USER CODE END LL_EXTI_LINE_12_RISING */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles ADC1 interrupt.
  */
void ADC1_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_IRQn 0 */

    /* Set flag to true */
    Flg.ADCCMPLT = 255;


    if(LL_ADC_IsActiveFlag_EOC(ADC1) != 0)
    	  {
    	    /* Clear flag ADC group regular end of unitary conversion */
    	    LL_ADC_ClearFlag_EOC(ADC1);

    	    /* Call interruption treatment function */
    	    ad_value=LL_ADC_REG_ReadConversionData12 (ADC1);
    	    LL_ADC_REG_StartConversion(ADC1); // RESTART THE NEXT CONVERSION

    	  }



  /* USER CODE END ADC1_IRQn 0 */
  /* USER CODE BEGIN ADC1_IRQn 1 */

  /* USER CODE END ADC1_IRQn 1 */
}

/**
  * @brief This function handles USART1 interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
	  usart1_interrupt_handler();
  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
