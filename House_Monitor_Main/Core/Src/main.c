/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lps25hb.h"
#include "dht11.h"
#include "eeprom.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SUPPLIED_VOLTAGE 3.3f

#define FIXED_PHOTORESISTOR_RESISTANCE 1000.0f
#define PHOTORESISTOR_EXPONENT 0.95f
#define PHOTORESISTOR_MULTIPLIER 7000000.0f

#define LIGHT_START_ADDR 0x00
#define LIGHT_END_ADDR 0x1F
#define TEMP_START_ADDR 0x20
#define TEMP_END_ADDR 0x3F
#define PRESSURE_START_ADDR 0x40
#define PRESSURE_END_ADDR 0x5F
#define HUMIDITY_START_ADDR 0x60
#define HUMIDITY_END_ADDR 0x7F

#define PRINT_MODE 0b100
#define STANDBY_MODE 0b010
#define MEASURE_MODE 0b001
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t current_light_waddr = LIGHT_START_ADDR;
static uint8_t current_temp_waddr = TEMP_START_ADDR;
static uint8_t current_press_waddr = PRESSURE_START_ADDR;
static uint8_t current_humi_waddr = HUMIDITY_START_ADDR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Redirecting chars in printf statements to USART2
int __io_putchar(int ch)
{
	if (ch == '\n')
		__io_putchar('\r');

	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return 1;
}

static float decode_light(uint16_t raw_value)
{
	return raw_value * 100.0f / 4096.0f;
}

static float decode_temperature(uint16_t raw_value)
{
	return raw_value / 100.0f;
}

static float decode_pressure(uint16_t raw_value)
{
	return raw_value / 10.0f;
}

static float decode_humidity(uint16_t raw_value)
{
	return raw_value / 100.0f;
}

static float merge_int_dec_part(const float integer_part, const float decimal_part)
{
	float result = integer_part;

	if (decimal_part <= 100)
		result += decimal_part / 100.0f;
	else
		result += decimal_part / 1000.0f;

	return result;
}

static HAL_StatusTypeDef Write_Measured_Data(const uint16_t phot, const uint16_t temp, const uint16_t press, const uint16_t humi)
{
	uint8_t phot_high = (phot >> 8) & 0xFF;
	uint8_t phot_low = phot & 0xFF;

	uint8_t temp_high = (temp >> 8) & 0xFF;
	uint8_t temp_low = temp & 0xFF;

	uint8_t press_high = (press >> 8) & 0xFF;
	uint8_t press_low = press & 0xFF;

	uint8_t humi_high = (humi >> 8) & 0xFF;
	uint8_t humi_low = humi & 0xFF;

	if (EEPROM_Write(current_light_waddr, (const void*)&phot_high, sizeof(phot_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_light_waddr + 1, (const void*)&phot_low, sizeof(phot_low)) != HAL_OK)
		return HAL_ERROR;

	current_light_waddr += 2;
	if (current_light_waddr > LIGHT_END_ADDR)
		current_light_waddr = LIGHT_START_ADDR;

	if (EEPROM_Write(current_temp_waddr, (const void*)&temp_high, sizeof(temp_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_temp_waddr + 1, (const void*)&temp_low, sizeof(temp_low)) != HAL_OK)
		return HAL_ERROR;

	current_temp_waddr += 2;
	if (current_temp_waddr > TEMP_END_ADDR)
		current_temp_waddr = TEMP_START_ADDR;

	if (EEPROM_Write(current_press_waddr, (const void*)&press_high, sizeof(press_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_press_waddr + 1, (const void*)&press_low, sizeof(press_low)) != HAL_OK)
		return HAL_ERROR;

	current_press_waddr += 2;
	if (current_press_waddr > PRESSURE_END_ADDR)
		current_press_waddr = PRESSURE_START_ADDR;

	if (EEPROM_Write(current_humi_waddr, (const void*)&humi_high, sizeof(humi_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_humi_waddr + 1, (const void*)&humi_low, sizeof(humi_low)) != HAL_OK)
		return HAL_ERROR;

	current_humi_waddr += 2;
	if (current_humi_waddr > HUMIDITY_END_ADDR)
		current_humi_waddr = HUMIDITY_START_ADDR;

	return HAL_OK;
}

static HAL_StatusTypeDef Read_Measured_Data(uint16_t* data, uint8_t addr)
{
	uint8_t buffor[2] = {0};

	if (EEPROM_Read(addr, (void*)buffor, sizeof(buffor)) != HAL_OK)
		return HAL_ERROR;

	*data = (uint16_t)buffor[0] << 8 | buffor[1];

	return HAL_OK;
}

static HAL_StatusTypeDef Take_Measurements(uint16_t* phot, uint16_t* temp, uint16_t* press, uint16_t* humi)
{
	uint8_t dht_vals[4] = {0};

	if (DHT11_Read(dht_vals) != HAL_OK)
		return HAL_ERROR;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	*phot = HAL_ADC_GetValue(&hadc1);

	float lps_temp = LPS_Read_Temp();
	float dht_temp = merge_int_dec_part((float)dht_vals[2], (float)dht_vals[3]);
	float average_temp = (lps_temp + dht_temp) / 2.0f;
	*temp = average_temp * 100;

	float pressure = LPS_Read_Pressure();
	*press = pressure * 10;

	float humidity = merge_int_dec_part((float)dht_vals[0], (float)dht_vals[1]);
	*humi = humidity * 100;

	return HAL_OK;
}

static HAL_StatusTypeDef Print_Measured_Data(uint8_t current_addr, uint8_t max_addr, float (*decode_fnc)(uint16_t))
{
	while (current_addr < max_addr) {
		uint16_t data = 0;

		if (Read_Measured_Data(&data, current_addr) != HAL_OK)
			return HAL_ERROR;

		printf("%.2f ", (*decode_fnc)(data));

		current_addr += 2;
	}
	printf("\n");

	return HAL_OK;
}

static void Diode_Signal(uint8_t bit_mask)
{
	HAL_GPIO_WritePin(PRINT_DIODE_GPIO_Port, PRINT_DIODE_Pin, bit_mask & 0x04);
	HAL_GPIO_WritePin(STANDBY_DIODE_GPIO_Port, STANDBY_DIODE_Pin, bit_mask & 0x02);
	HAL_GPIO_WritePin(MEASURE_DIODE_GPIO_Port, MEASURE_DIODE_Pin, bit_mask & 0x01);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  //Initialize ADC for photoresistor reading
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADCEx_Calibration_Start(&hadc2);

  // Initialize LPS25HB sensor and timer
  // necessary for its functioning
  HAL_TIM_Base_Start(&htim2);
  if (LPS_Init() != HAL_OK)
	  Error_Handler();

  //LPS_Calibrate(32);

  // Clear EEPROM memory
  if (EEPROM_Reset(0) != HAL_OK)
  	  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_IWDG_Refresh(&hiwdg);
  HAL_Delay(1000);
  while (1)
  {
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  uint16_t pot_reading = HAL_ADC_GetValue(&hadc2);

	  if (pot_reading < 1000) { // Measuring mode

		  Diode_Signal(MEASURE_MODE);

		  uint16_t phot = 0, temp = 0, press = 0, humi = 0;

		  if (Take_Measurements(&phot, &temp, &press, &humi) != HAL_OK)
			  Error_Handler();

		  if (Write_Measured_Data(phot, temp, press, humi) != HAL_OK)
			  Error_Handler();

	  } else if (pot_reading > 3000) { // Reading mode

		  Diode_Signal(PRINT_MODE);

		  printf("LIGHT: ");
		  if (Print_Measured_Data(LIGHT_START_ADDR, LIGHT_END_ADDR, &decode_light) != HAL_OK)
			  Error_Handler();

		  printf("TEMPERATURE: ");
		  if (Print_Measured_Data(TEMP_START_ADDR, TEMP_END_ADDR, &decode_temperature) != HAL_OK)
			  Error_Handler();

		  printf("PRESSURE: ");
		  if (Print_Measured_Data(PRESSURE_START_ADDR, PRESSURE_END_ADDR, &decode_pressure) != HAL_OK)
			  Error_Handler();

		  printf("HUMIDITY: ");
		  if (Print_Measured_Data(HUMIDITY_START_ADDR, HUMIDITY_END_ADDR, &decode_humidity) != HAL_OK)
			  Error_Handler();

		  printf("END\n");
	  } else {
		  Diode_Signal(STANDBY_MODE);
	  }

	  HAL_IWDG_Refresh(&hiwdg);
	  HAL_Delay(1000);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
