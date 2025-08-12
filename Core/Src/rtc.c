/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
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
#include "rtc.h"

/* USER CODE BEGIN 0 */

void RTC_SetDateTime(void)
{
    LL_RTC_DisableWriteProtection(RTC);

    // Desabilita o RTC para permitir configurações
    LL_RTC_EnterInitMode(RTC);

    // Configura hora

    LL_RTC_TimeTypeDef time = {0};
    time.Hours   = __LL_RTC_CONVERT_BIN2BCD(HOUR_INITIAL);
    time.Minutes = __LL_RTC_CONVERT_BIN2BCD(MINUTE_INITIAL);
    time.Seconds = __LL_RTC_CONVERT_BIN2BCD(0);
    LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &time);


    // Configura data



    LL_RTC_DateTypeDef date = {0};
    date.WeekDay = LL_RTC_WEEKDAY_MONDAY;
    date.Month   = __LL_RTC_CONVERT_BIN2BCD(MES_INITIAL);
    date.Day     = __LL_RTC_CONVERT_BIN2BCD(DIA_INITIAL);
    date.Year    = __LL_RTC_CONVERT_BIN2BCD(ANO_INITIAL);
    LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &date);


    // Sai do modo de inicialização

    LL_RTC_ExitInitMode(RTC);

    LL_RTC_EnableWriteProtection(RTC);
}


void RTC_Set_Alarm(void)
{
    LL_RTC_DisableWriteProtection(RTC);
    LL_RTC_ALMA_Disable(RTC);
    while (!LL_RTC_IsActiveFlag_ALRAW(RTC));

    uint8_t hours   = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetHour(RTC));
    uint8_t minutes = __LL_RTC_CONVERT_BCD2BIN(LL_RTC_TIME_GetMinute(RTC));


    minutes += SLEEP_TIME;
    if (minutes >= 60) {
        minutes -= 60;
        hours = (hours + 1) % 24;
    }

    LL_RTC_ALMA_ConfigTime(RTC, LL_RTC_FORMAT_BCD,
                           __LL_RTC_CONVERT_BIN2BCD(hours),
                           __LL_RTC_CONVERT_BIN2BCD(minutes),
                           0);

    LL_RTC_ALMA_SetMask(RTC, LL_RTC_ALMA_MASK_DATEWEEKDAY);
    LL_RTC_ClearFlag_ALRA(RTC);
    LL_RTC_EnableIT_ALRA(RTC);
    LL_RTC_ALMA_Enable(RTC);

    // Habilita EXTI linha 17

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_19);
    LL_EXTI_EnableEvent_0_31(LL_EXTI_LINE_19);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_19);



    LL_RTC_EnableWriteProtection(RTC);
}

/* USER CODE END 0 */

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  LL_RTC_InitTypeDef RTC_InitStruct = {0};
  LL_RTC_TimeTypeDef RTC_TimeStruct = {0};
  LL_RTC_DateTypeDef RTC_DateStruct = {0};
  LL_RTC_AlarmTypeDef RTC_AlarmStruct = {0};

  if(LL_RCC_GetRTCClockSource() != LL_RCC_RTC_CLKSOURCE_LSI)
  {
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSI);
  }

  /* Peripheral clock enable */
  LL_RCC_EnableRTC();
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_RTC);

  /* RTC interrupt Init */
  NVIC_SetPriority(RTC_IRQn, 0);
  NVIC_EnableIRQ(RTC_IRQn);

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  RTC_InitStruct.HourFormat = LL_RTC_HOURFORMAT_24HOUR;
  RTC_InitStruct.AsynchPrescaler = 127;
  RTC_InitStruct.SynchPrescaler = 255;
  LL_RTC_Init(RTC, &RTC_InitStruct);
  RTC_TimeStruct.Hours = 0x0;
  RTC_TimeStruct.Minutes = 0x0;
  RTC_TimeStruct.Seconds = 0x0;

  LL_RTC_TIME_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_TimeStruct);
  RTC_DateStruct.WeekDay = LL_RTC_WEEKDAY_MONDAY;
  RTC_DateStruct.Month = LL_RTC_MONTH_JANUARY;
  RTC_DateStruct.Day = 0x1;
  RTC_DateStruct.Year = 0x0;

  LL_RTC_DATE_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_DateStruct);

  /** Enable the Alarm A
  */
  RTC_AlarmStruct.AlarmTime.Hours = 0x0;
  RTC_AlarmStruct.AlarmTime.Minutes = 0x0;
  RTC_AlarmStruct.AlarmTime.Seconds = 0x0;
  RTC_AlarmStruct.AlarmMask = LL_RTC_ALMA_MASK_NONE;
  RTC_AlarmStruct.AlarmDateWeekDaySel = LL_RTC_ALMA_DATEWEEKDAYSEL_DATE;
  RTC_AlarmStruct.AlarmDateWeekDay = 0x1;
  LL_RTC_ALMA_Init(RTC, LL_RTC_FORMAT_BCD, &RTC_AlarmStruct);
  LL_RTC_EnableIT_ALRA(RTC);
  LL_RTC_DisableAlarmPullUp(RTC);
  /* USER CODE BEGIN RTC_Init 2 */
  RTC_SetDateTime();
  RTC_Set_Alarm();
  /* USER CODE END RTC_Init 2 */

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
