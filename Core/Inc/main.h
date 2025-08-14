/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"

#include "stm32c0xx_ll_adc.h"
#include "stm32c0xx_ll_rcc.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_ll_system.h"
#include "stm32c0xx_ll_exti.h"
#include "stm32c0xx_ll_cortex.h"
#include "stm32c0xx_ll_utils.h"
#include "stm32c0xx_ll_pwr.h"
#include "stm32c0xx_ll_dma.h"
#include "stm32c0xx_ll_rtc.h"
#include "stm32c0xx_ll_usart.h"
#include "stm32c0xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct Flags
{
  uint8_t ADCCMPLT;
}flag_t;


extern flag_t Flg;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TX_Pin LL_GPIO_PIN_14
#define TX_GPIO_Port GPIOC
#define ntc_Pin LL_GPIO_PIN_8
#define ntc_GPIO_Port GPIOA
#define memory_print_Pin LL_GPIO_PIN_12
#define memory_print_GPIO_Port GPIOA
#define memory_print_EXTI_IRQn EXTI4_15_IRQn
#define alim_ntc_Pin LL_GPIO_PIN_13
#define alim_ntc_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

//ADC PARAMETERS


#define RECHARGE_START 3000 		//START CHARGE MS
#define TEMPERATURE_DEFAULT 26.0f		//26 CELSIUS DEGREE
#define RECHARGE_DEFAULT 10 		//RECHARGE COUNT
#define RECHARGE_PERIOD 25 		//RECHARGE PERIOD MS
#define RECHARGE_CYCLE_DELAY 100 	//DELAY MS
#define RECHARGE_CYCLE_DELAY_BEFORE 150 	//DELAY MS
#define PRE_MEASURE_DELAY 200 	//DELAY MS
#define RESISTOR_VALUE 20000 	//RESISTOR VALUE IN OHMS
#define NTC_TEMPERATURE_ADJUST 0 	//


#define FATOR_A_NTC 1.18
#define FATOR_B_NTC 1.93


#define IMPULSE_LIMIT 75 // 50 cycles/sec  75 = 1.5sec

//MEAN
#define QUANT_DATA 12 //40 QUANTITY OF MEASURES TO CREATE THE MEAN
#define TRIM_PERCENTAGE 20 // 20%
#define CH8_RAW_LIMIT 2048
#define TEST_COUNTER_LIMIT 300 //ON-OFF CICLE

#define ON  1
#define OFF 0

#define POWER_ON OFF
#define POWER_OFF ON


//#define PRINT_A //if defined detailed
#define PRINT_B   //if defined basic
#define TEST_MODE OFF //OFF=NORMAL WORK
#define PRINT_MODE ON //OFF=DISABLE ALL PRINT

typedef struct relay_ {
      uint8_t status;
      uint8_t re1charge;

} relay_st;



#define INITIAL_PAGE 14 //PAGE 14 = ADDRESS 0X8007000
#define PERIOD 3 //MINUTOS
#define QUANT_REGISTRY 2 //QUANTITY OF MEASURES TO SAVE, MAX 128 =512BYTES
typedef struct __attribute__((packed, aligned(8))) {
      float temp[QUANT_REGISTRY];
} flash_st;



typedef struct time_struct_ {
  uint8_t bcd_ano;
  uint8_t bcd_dia;
  uint8_t bcd_mes;
  uint8_t bcd_horas;
  uint8_t bcd_minutos;
  uint8_t bcd_seconds;
} time_struct;


typedef struct sensor_item_ {
  float temp;
  float vdda_real;
} sensor_item;

#define  FAST_TEST_MODE OFF

typedef struct queue_item_ {
  float temp;
  float vdda_real;
  time_struct timestamp;
} queue_item;

#define SLEEP_TIME 10  //how many minutes between measures
#define ANO_INITIAL 25
#define DIA_INITIAL 1
#define MES_INITIAL 1
#define HOUR_INITIAL 00
#define MINUTE_INITIAL 00

#define QUEUE_SIZE 240  // Tamanho máximo da fila, don't change it
typedef struct {
	queue_item buffer[QUEUE_SIZE]; // Array de elementos do tipo string_tcp
    int head;                      // Índice do primeiro elemento da fila
    int tail;                      // Índice do próximo elemento disponível
    int count;                     // Número de elementos na fila
} fifo_queue;



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
