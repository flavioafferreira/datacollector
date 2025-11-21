/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  Normal current sleep consumption (sleeping) = 110uA
  The battery CR2032 (220mAh) will work for 80days (2000 hours)

  STM32C011J6Mx works from 1.7V up to 3.6V
  connections
  GND---ntc10k--PA8(ntc)--res20k-----PA13(alim_ntc)
  PA12(memory_print)--switch---VDD
  PC14(TX)-->UART TX 1200BPS
  PC15--SWDCLK
  PC13--SWDIO
  PF2 --RESET
  PIN2--VDD
  PIN3--VSS GND
  Don't need external Crystal. It uses the internal RC.
  This schematics should consider 2 capacitors 0.1uF and 10uf between GND and VDD near the chip
  and 1uF on Reset Port NRST
  When press the switch send the latest 240(max) readings to UART TX 1200bps and clean the buffer
  You can configure the interval between readings from 1 to 60 minutes on main.h
  "#define SLEEP_TIME 60  //how many minutes between measures"
  The readings are stored on internal RAM, then if you turn off the batteries,
  will lost the measures.

  When press the switch will output the readings. In this test the interval was configured as 1 minute.
  You can setup the initial date and time and compile.

04/08/25 12:38:04 T:22.2 3.1V
04/08/25 12:39:04 T:22.4 3.1V
04/08/25 12:40:04 T:22.2 3.1V
04/08/25 12:41:04 T:22.2 3.1V
04/08/25 12:42:04 T:22.1 3.1V
04/08/25 12:43:04 T:22.1 3.1V
04/08/25 12:44:04 T:22.1 3.1V
04/08/25 12:45:04 T:22.8 3.1V
04/08/25 12:46:04 T:22.5 3.1V
04/08/25 12:47:04 T:22.6 3.1V



  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "math.h"
#include "stm32c0xx_ll_bus.h"
#include "stm32c0xx_hal.h"
#include "string.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// QUEUE DEFINITION
 //fifo to TCP
fifo_queue q;
queue_item item;

//time
time_struct timestamp;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//VARIABLE FOR AD CONVERSION
uint16_t ad_value =0;
uint32_t qty_of_samples=1;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
relay_st relay;
relay_st relay_on_off(uint8_t control);
uint8_t triac_zc_on=OFF;
static uint16_t vref_raw,temp_raw,vdda_raw,ch12_raw;

flash_st flash_variable;
uint8_t calibration_completed=OFF;
uint32_t ext_cnt=0;
volatile uint8_t print_memory=OFF;
volatile uint8_t it_ready=ON;
char *str_cmd[QTY_CMD] = CMD_DEFINITION;
extern char rx_buffer[RX_BUFFER_SIZE];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct AdcValues{
  uint16_t Raw[2]; /* Raw values from ADC */
  double IntSensTmp; /* Temperature */
}adcval_t;
adcval_t Adc;
flag_t Flg = {0, };

uint32_t minutos_atual=0;

//redirect the printf to usart1
int _write(int file, char *ptr, int len) {
    for (int i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));  // Wait for the buffer to be empty
        LL_USART_TransmitData8(USART1, ptr[i]);  // Send a character
    }
    while (!LL_USART_IsActiveFlag_TC(USART1));  // Wait for the transmission to finish
    return len;
}


void exec_cmd(uint8_t cmd){
       switch(cmd){

       case 0:
    	   printf("CMD MEM\n\r");
    	   break;
       case 1:
    	   break;
       case 2:
    	   //Init_flash_logging();
    	   //Flash_EraseLogVariable(FLASH_PAGE_8,8);
   		   //flash_variable.working_timestamp=0;
   		   //flash_variable.index=0;
   		   //Flash_WriteRelayVariable(&flash_variable,FLASH_USER_START_ADDR);
    	   //printf("flash reseted\n\r");
    	   break;
       case 3:
    	   break;
       case 4:
    	   //flash_logg_update();
           //printf("force updated\n\r");
    	   break;
       default:

       }

}



void print_float(float number) {
    int32_t mantissa = (int32_t)number;  // Integer part
    float intermediate = number - mantissa;
    int32_t expoente = (int32_t)(intermediate * 100000); // Decimal part (5 digits)

    // Ensure the exponent is positive
    if (expoente < 0) expoente *= -1;

    // Correct printing ensuring 5 digits in the decimal part
    printf("%ld.%05ld", mantissa, expoente);
}

void print_float_1dec(float number) {
    int32_t mantissa = (int32_t)number;  // Integer part
    float intermediate = number - mantissa;
    int32_t expoente = (int32_t)(intermediate * 100000); // Decimal part (5 digits)

    // Ensure the exponent is positive
    if (expoente < 0) expoente *= -1;

    while (expoente >= 10) {
    	expoente /= 10;
        }
    // Correct printing ensuring 1 digit
    printf("%ld.%ld", mantissa, expoente);
}


void sort(float* data, int n) {
    for (int i = 0; i < n - 1; ++i) {

        for (int j = 0; j < n - i - 1; ++j) {
            if (data[j] > data[j + 1]) {
                float temp = data[j];
                data[j] = data[j + 1];
                data[j + 1] = temp;
            }
        }
    }
}

float trimmed_mean(float* data, int n, float trim_percentage) {
    int trim_count = (int)(n * trim_percentage / 100);
    sort(data, n);
    float sum = 0.0;
    for (int i = trim_count; i < n - trim_count; ++i) {
        sum += data[i];
    }
    int remaining_elements = n - 2 * trim_count;
    float trimmed_mean_value = sum / remaining_elements;
    return trimmed_mean_value;
}


void ADC_Init(void)
{
    // Enable temperature sensor and VrefInt
    LL_ADC_SetCommonPathInternalCh(ADC1_COMMON, LL_ADC_PATH_INTERNAL_TEMPSENSOR | LL_ADC_PATH_INTERNAL_VREFINT);


    // ADC configuration
    LL_ADC_SetResolution(ADC1, LL_ADC_RESOLUTION_12B);
    LL_ADC_SetDataAlignment(ADC1, LL_ADC_DATA_ALIGN_RIGHT);
    LL_ADC_SetLowPowerMode(ADC1, LL_ADC_LP_MODE_NONE);

    // Configure channel sequence: Temperature (RANK 1) and VrefInt (RANK 2)
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_TEMPSENSOR);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_2, LL_ADC_CHANNEL_VREFINT);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_3, LL_ADC_CHANNEL_VDDA);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_4, LL_ADC_CHANNEL_12);

    //LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);

    // Set sampling times for higher accuracy
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_TEMPSENSOR, LL_ADC_SAMPLINGTIME_160CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VREFINT, LL_ADC_SAMPLINGTIME_160CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_VDDA, LL_ADC_SAMPLINGTIME_160CYCLES_5);
    LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_12, LL_ADC_SAMPLINGTIME_160CYCLES_5);


    // Enable the ADC
    LL_ADC_Enable(ADC1);

    // Wait for stabilization
    while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
}


static uint16_t ADC_ReadSingleChannel(uint32_t channel)
{
    uint32_t timeout;

    // garante ADC ligado
    if (!LL_ADC_IsEnabled(ADC1)) {
        LL_ADC_Enable(ADC1);
        while (!LL_ADC_IsActiveFlag_ADRDY(ADC1));
    }

    // habilita canais internos quando necessário
    if (channel == LL_ADC_CHANNEL_TEMPSENSOR ||
        channel == LL_ADC_CHANNEL_VREFINT  ||
        channel == LL_ADC_CHANNEL_VDDA)
    {
        LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
            LL_ADC_PATH_INTERNAL_TEMPSENSOR |
            LL_ADC_PATH_INTERNAL_VREFINT  );
    }

    // sequenciador com 1 rank só
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, channel);

    // espera CCRDY (igual CubeMX faz)
    timeout = 100000;
    while (!LL_ADC_IsActiveFlag_CCRDY(ADC1) && --timeout);
    LL_ADC_ClearFlag_CCRDY(ADC1);

    // limpa flags antigas
    LL_ADC_ClearFlag_EOC(ADC1);
    LL_ADC_ClearFlag_EOS(ADC1);
    LL_ADC_ClearFlag_OVR(ADC1);

    // start conversão
    LL_ADC_REG_StartConversion(ADC1);

    // espera fim da conversão (agora é 1 canal só, EOC vem sempre)
    timeout = 1000000;
    while (!LL_ADC_IsActiveFlag_EOC(ADC1) && --timeout);
    if (timeout == 0) {
        printf("ADC timeout ch=%lu ISR=0x%08lx\n\r", channel, ADC1->ISR);
        return 0;
    }

    return LL_ADC_REG_ReadConversionData12(ADC1);
}

void ADC_Read_Temperature_And_Vref(uint16_t *temp_raw,
                                   uint16_t *vref_raw,
                                   uint16_t *vdda_raw,
                                   uint16_t *ch12_raw)
{
    *temp_raw = ADC_ReadSingleChannel(LL_ADC_CHANNEL_TEMPSENSOR);
    *vref_raw = ADC_ReadSingleChannel(LL_ADC_CHANNEL_VREFINT);
    *vdda_raw = ADC_ReadSingleChannel(LL_ADC_CHANNEL_VDDA);
    *ch12_raw = ADC_ReadSingleChannel(LL_ADC_CHANNEL_12);
}




float getTemperatureFromVoltage(float v_adc, float vdd) {
    // Temperatures in degrees Celsius
    static const float temps[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};

    // Typical resistance of 10k NTC for each temperature (Ω)
    static const float r_ntc[] = {32650, 25200, 19560, 15300, 12100, 9600, 7700, 6200, 5000, 4050, 3300};

    // Expected voltage for each point (computed at runtime only in the 1st call)
    float volts[11];

    for (int i = 0; i < 11; i++) {
            volts[i] = vdd * (r_ntc[i] / (r_ntc[i] + RESISTOR_VALUE));
    }

    // Linear interpolation between two points in the table
    for (int i = 0; i < 10; i++) {
        if (v_adc <= volts[i] && v_adc >= volts[i + 1]) {
            float t = temps[i];
            float tNext = temps[i + 1];
            float v1 = volts[i];
            float v2 = volts[i + 1];
            return t + (v1 - v_adc) * (tNext - t) / (v1 - v2);
        }
    }

    // Out of range — optional: extrapolation or return error
    return -100.0f;
}




sensor_item Get_Temperature(void)
{

	sensor_item data;

    #define VREF_VOLTAGE_CALIBRATION 3.3f //AT THE MANUAL IS WROTE 3.0VOLTS Raw data acquired at a temperature of 30 °C (± 5 °C), VDDA = VREF+ = 3.0 V (± 10 mV)
    #define AVG_SLOPE (0.00253f) //  volts/grau
    #define VREFINT_CAL  (*((uint16_t*)0x1FFF756A)) // Valor de calibração de referência
    #define TS_CAL_30      (*((uint16_t*)0x1FFF7568)) // ADC value @ 30°C VDDA 3.3V

    #define TEMPERATURE_CALIBRATION 16 // ADJUST HERE THE CORRECT TEMPERATURE OF CALIBRATION
    //#define CALIBRATION_ERROR_ADJUST -5 //degree
    #define VOLTAGE_ERROR_ADJUST 0.022
	//COMPENSATION
    #define FATOR_A 3.883f
    #define FATOR_B -11.60f



	float Temperature=0,Internal_Temperature=0, V_Temp=0,V_Cal_30=0,V_Ad12=0;
	//static uint16_t vref_raw,temp_raw,vdda_raw,ch12_raw;
    float vdda_real=0;
    ADC_Read_Temperature_And_Vref(&temp_raw,&vref_raw,&vdda_raw,&ch12_raw);

#ifdef PRINT_A
	printf("temp_raw: %d, ref_raw: %d, vdda_raw: %d, ch12_raw: %d\n\r", temp_raw, vref_raw,vdda_raw,ch12_raw);
    printf("VREFINT_CAL:%d\n\r",VREFINT_CAL);
    printf("TS_CAL_30:%d\n\r",TS_CAL_30);
#endif

	//vdda_real= VOLTAGE_ERROR_ADJUST+ (VREF_VOLTAGE_CALIBRATION * (*VREFINT_CAL_ADDR)) / (float)vref_raw;  //resultado em Volts

	vdda_real = VOLTAGE_ERROR_ADJUST +( (VREF_VOLTAGE_CALIBRATION * (float)(*VREFINT_CAL_ADDR)) / (float)vref_raw);
    V_Cal_30 = (float)TS_CAL_30 * (VREF_VOLTAGE_CALIBRATION / 4095.0f);  //how many volts the sensor had at 30 degrees with VDDA at 3.3V with an error of ±5 degrees
    V_Temp   = (float)temp_raw *  (vdda_real / 4095.0f);  //how many volts the sensor is actually reading
    V_Ad12     = (float)ch12_raw *  (vdda_real / 4095.0f);
    Internal_Temperature = ((V_Temp -V_Cal_30) / AVG_SLOPE)+TEMPERATURE_CALIBRATION;
    Internal_Temperature = Internal_Temperature - (FATOR_A * vdda_real + FATOR_B);

#ifdef PRINT_A
    printf("vdda_real:");print_float(vdda_real);printf("\n\r");
    printf("V_Cal_30:");print_float(V_Cal_30);printf("\n\r");
    printf("V_Temp:");print_float(V_Temp);printf("\n\r");
    printf("Internal_Temp:");print_float(Internal_Temperature);printf("\n\r");
    printf("V_Ad12:");print_float(V_Ad12);printf("\n\r");
    printf("T_NTC:");print_float(getTemperatureFromVoltage(V_Ad12,vdda_real));printf("\n\r");

#endif

    //printf("T_NTC:");print_float(getTemperatureFromVoltage(V_Ad8,vdda_real));printf("\n\r");
    Temperature=getTemperatureFromVoltage(V_Ad12,vdda_real)+NTC_TEMPERATURE_ADJUST; //from NTC
    //deploy a factor to change the temperature using the vdda_real as parameter
    //print_float(Temperature);printf("\n\r");

    data.temp=Temperature;
    data.vdda_real=vdda_real;

    return data;  // Returns the integer temperature in °C

}


HAL_StatusTypeDef Flash_Write(flash_st *data,uint8_t page_number)
{
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t PageError = 0;
    uint32_t flashAddress = (page_number * FLASH_PAGE_SIZE)+0x08000000; //2kbytes/page

    // Unlock the Flash
    HAL_FLASH_Unlock();

    // Configuration to erase the page where data will be written
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.Page = 14;
    eraseInitStruct.NbPages = 1;

    status = HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    // Write the entire structure, 8 bytes at a time
    uint64_t *data_ptr = (uint64_t *)data;
    for (uint32_t i = 0; i < sizeof(flash_st) / sizeof(uint64_t); i++)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, flashAddress, data_ptr[i]);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
        flashAddress += sizeof(uint64_t);
    }

    // Lock the Flash again
    HAL_FLASH_Lock();

    return HAL_OK;
}

HAL_StatusTypeDef Flash_Erase(uint8_t page_number,uint8_t qty_pages){
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t PageError = 0;

    // Unlock the flash memory for write access
    HAL_FLASH_Unlock();

    eraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.NbPages     = qty_pages;
    //FLASH_PAGE_NB
    eraseInitStruct.Page = page_number;

    // Erase the flash page
    status = HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }


    // Lock the flash memory after writing
    HAL_FLASH_Lock();

    return HAL_OK;
}




void Flash_Read(flash_st *data,uint8_t page_number)
{
	uint32_t flashAddress_data=(page_number*FLASH_PAGE_SIZE)+0x08000000;
    memcpy(data, (void *)flashAddress_data, sizeof(flash_st));
}



sensor_item temperature_level_control(void){

	sensor_item data;
	//float temperature=0;
	float vdda;
	float temp[QUANT_DATA];
	uint32_t i=0;

   	while (i < QUANT_DATA){
   	  data =Get_Temperature();
      temp[i] = data.temp; //CALIBRATION ADJUSTED
      vdda= data.vdda_real;
      //temp[i] = Get_Temperature() ; //WITHOUT CALIBRATION ADJUST

	  i++;
  	}
  	//relay=relay_on_off(relay_present_status);

    data.temp=trimmed_mean(temp, QUANT_DATA, TRIM_PERCENTAGE);
    data.temp=FATOR_A_NTC*data.temp+FATOR_B_NTC;
    data.vdda_real=vdda;
    return data;
}


//QUEUE FUNCTIONS
void init_queue(fifo_queue *q)
{
    q->head = 0;
    q->tail = 0;
    q->count = 0;
}

//FIFO
int enqueue_FIFO(fifo_queue *q, queue_item item)
{
    if (q->count == QUEUE_SIZE)
    {
        // Queue full
        return -1;
    }

    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;

    return 0; // Success
}


//circular queue with overwrite
int enqueue(fifo_queue *q, queue_item item)
{
    if (q->count == QUEUE_SIZE)
    {
        // Queue full: discard the oldest (head)
        q->head = (q->head + 1) % QUEUE_SIZE;
        q->count--; // since we are going to insert a new one, we decrement first
    }

    q->buffer[q->tail] = item;
    q->tail = (q->tail + 1) % QUEUE_SIZE;
    q->count++;

    return 0; // Success
}



int dequeue(fifo_queue *q, queue_item *item)
{
    if (q->count == 0)
    {
        // Queue empty
        return -1;
    }

    *item = q->buffer[q->head];
    q->head = (q->head + 1) % QUEUE_SIZE;
    q->count--;

    return 0; // Success
}
int is_empty(fifo_queue *q)
{
    return (q->count == 0);
}
int is_full(fifo_queue *q)
{
    return (q->count == QUEUE_SIZE);
}


void Ler_Tempo_RTC(uint8_t* horas, uint8_t* minutos)
{
    // Make sure you are accessing correctly (in RTC read sequence)
    LL_RTC_DisableWriteProtection(RTC);

    // Read the values in BCD
    uint8_t bcd_horas   = LL_RTC_TIME_GetHour(RTC);
    uint8_t bcd_minutos = LL_RTC_TIME_GetMinute(RTC);

    // Convert to decimal (BIN)
    *horas   = __LL_RTC_CONVERT_BCD2BIN(bcd_horas);
    *minutos = __LL_RTC_CONVERT_BCD2BIN(bcd_minutos);

    LL_RTC_EnableWriteProtection(RTC);
}


void button_pressed_test(void){
    uint32_t item_nro=1;
    //if press the button, print the queue
	if (print_memory==ON){
	   	print_memory=OFF;
		SystemClock_Config(); // Need to restore HSI + PLL if needed
		printf("Queue:\n\r");
    	while (dequeue(&q, &item) == 0) {

    		printf("%04ld %02d/%02d/%02d %02d:%02d:%02d ",item_nro,
    		       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_dia),
    		       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_mes),
    		       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_ano),
    		       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_horas),
    		       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_minutos),
    		       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_seconds));


    		printf("T:");print_float_1dec(item.temp);
    		printf(" ");print_float_1dec(item.vdda_real);printf("V\n\r");

    		item_nro++;

        }
    	printf("Queue End:\n\r");
    	//printf("T:");print_float_1dec(flash_variable.temp[i]);printf("\n\r");
    }





}

void Enter_Stop_Mode_old(void)
{
    // Turn off SysTick (optional if not used)
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    LL_USART_Disable(USART1);
    LL_ADC_Disable(ADC1);
    LL_DBGMCU_DisableDBGStopMode();



    // Configure entry into STOP mode
    LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
    LL_LPM_EnableDeepSleep(); // SLEEPDEEP = 1

    __WFI();  // Enter STOP mode (wait for interrupt)

    // Upon wake-up, reconfigure clock if necessary
    SystemClock_Config(); // Need to restore HSI + PLL if needed

    MX_ADC1_Init();
    MX_USART1_UART_Init();
    ADC_Init();

}


void Enter_Stop_Mode(void)
{
    // ensure idle UART
    while (!LL_USART_IsActiveFlag_TC(USART1)) {}

    // Turn off SysTick (if HAL delays are not used while sleeping)
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    // Turn off peripherals that you will reinitialize later
    LL_USART_Disable(USART1);
    LL_ADC_Disable(ADC1);
    LL_DBGMCU_DisableDBGStopMode();

    // CLEAR any pending flags that would wake up immediately
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_12); // button
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_19); // RTC ALARM

    // Enter STOP0
    LL_PWR_SetPowerMode(LL_PWR_MODE_STOP0);
    LL_LPM_EnableDeepSleep();
    __WFI();

    // Upon waking: (optional) exit 'logical' deep sleep
    // LL_LPM_EnableSleep();

    // Restore the clock and SysTick
    SystemClock_Config();  // HSI + divisores + HAL_InitTick
    // Reinit only what you turned off
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    ADC_Init();
}


void cmd_analise_task(void){
	int result;
        uint8_t i=0;
		while (i<QTY_CMD){
		 result = strcmp(str_cmd[i], rx_buffer);
		 if (!result)exec_cmd(i);
	     i++;
		}

}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */




  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  init_queue(&q);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  MX_USART1_UART_Init_New();
  ADC_Init();

   printf("\n\r###START##SAMPLE EACH %02d MINUTE, %03d SAMPLES\n\r",SLEEP_TIME,QUEUE_SIZE);
  /*

  if(PRINT_MODE)printf("\n\r\n\r<MEMORY>\n\r");
  Flash_Read(&flash_variable,current_flash_page);
  while(i<QUANT_REGISTRY){
  	  printf("R:%03ld ",i);
   printf("T:");print_float_1dec(flash_variable.temp[i]);printf("\n\r");
	  i++;
  }
  if (!(LL_GPIO_IsInputPinSet(memory_GPIO_Port, memory_Pin))){
      Flash_Erase(current_flash_page,1);
      if(PRINT_MODE)printf("\n\r\n\r<Mem Erased>\n\r");
  }
  if(PRINT_MODE)printf("\n\r\n\r<Ready>\n\r");
 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   it_ready=ON;
   sensor_item data;
  while (1)
  {
	Pin_Config_NTC_Alim();
	LL_GPIO_SetOutputPin(alim_ntc_GPIO_Port, alim_ntc_Pin);
	data=temperature_level_control();

	item.temp=data.temp;
	item.vdda_real=data.vdda_real;
	LL_GPIO_ResetOutputPin(alim_ntc_GPIO_Port, alim_ntc_Pin);
	Pin_Config_NTC_Disable();

	item.timestamp.bcd_dia=LL_RTC_DATE_GetDay(RTC);
	item.timestamp.bcd_mes=LL_RTC_DATE_GetMonth(RTC);
	item.timestamp.bcd_ano=LL_RTC_DATE_GetYear(RTC);
	item.timestamp.bcd_horas=LL_RTC_TIME_GetHour(RTC);
	item.timestamp.bcd_minutos=LL_RTC_TIME_GetMinute(RTC);
	item.timestamp.bcd_seconds=LL_RTC_TIME_GetSecond(RTC);
	enqueue(&q, item);

	printf("%06ld %02d/%02d/%02d %02d:%02d:%02d ",qty_of_samples,
	       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_dia),
	       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_mes),
	       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_ano),
	       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_horas),
	       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_minutos),
	       __LL_RTC_CONVERT_BCD2BIN(item.timestamp.bcd_seconds));


	printf("T:");print_float_1dec(item.temp);
	printf(" ");print_float_1dec(item.vdda_real);printf("V\n\r");

	cmd_analise_task(); //verify input commands

	qty_of_samples++;
     if(!FAST_TEST_MODE)Enter_Stop_Mode();
     button_pressed_test();
     /* clear bounce before rearm */
     if (it_ready==OFF){
      LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_12);
      NVIC_ClearPendingIRQ(EXTI4_15_IRQn);
      LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_12);
      it_ready=ON;
     }
     if(FAST_TEST_MODE)LL_mDelay(1000);
     RTC_Set_Alarm();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  LL_RCC_HSI_SetCalibTrimming(64);
  LL_RCC_SetHSIDiv(LL_RCC_HSI_DIV_64);
  /* LSI configuration and activation */
  LL_RCC_LSI_Enable();
  while(LL_RCC_LSI_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_HCLK_DIV_8);

  /* Sysclk activation on the HSI */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(93750);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
