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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t current_light_addr = LIGHT_START_ADDR;
uint8_t current_temp_addr = TEMP_START_ADDR;
uint8_t current_press_addr = PRESSURE_START_ADDR;
uint8_t current_humi_addr = HUMIDITY_START_ADDR;
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

static void calculate_photoresistor_values(float* lux_level, float* light_percentage)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	float photoresistor_voltage = HAL_ADC_GetValue(&hadc1) * SUPPLIED_VOLTAGE / 4096.0f;
	float photoresistor_resistance = FIXED_PHOTORESISTOR_RESISTANCE * (SUPPLIED_VOLTAGE / photoresistor_voltage - 1);

	*lux_level = PHOTORESISTOR_MULTIPLIER / pow(photoresistor_resistance, PHOTORESISTOR_EXPONENT);
	*light_percentage = photoresistor_voltage * 100.0f / SUPPLIED_VOLTAGE;
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

	if (EEPROM_Write(current_light_addr, (const void*)&phot_high, sizeof(phot_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_light_addr + 1, (const void*)&phot_low, sizeof(phot_low)) != HAL_OK)
		return HAL_ERROR;

	current_light_addr += 2;
	if (current_light_addr > LIGHT_END_ADDR)
		current_light_addr = LIGHT_START_ADDR;

	if (EEPROM_Write(current_temp_addr, (const void*)&temp_high, sizeof(temp_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_temp_addr + 1, (const void*)&temp_low, sizeof(temp_low)) != HAL_OK)
		return HAL_ERROR;

	current_temp_addr += 2;
	if (current_temp_addr > TEMP_END_ADDR)
		current_temp_addr = TEMP_START_ADDR;

	if (EEPROM_Write(current_press_addr, (const void*)&press_high, sizeof(press_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_press_addr + 1, (const void*)&press_low, sizeof(press_low)) != HAL_OK)
		return HAL_ERROR;

	current_press_addr += 2;
	if (current_press_addr > PRESSURE_END_ADDR)
		current_press_addr = PRESSURE_START_ADDR;

	if (EEPROM_Write(current_humi_addr, (const void*)&humi_high, sizeof(humi_high)) != HAL_OK)
		return HAL_ERROR;

	if (EEPROM_Write(current_humi_addr + 1, (const void*)&humi_low, sizeof(humi_low)) != HAL_OK)
		return HAL_ERROR;

	current_humi_addr += 2;
	if (current_humi_addr > HUMIDITY_END_ADDR)
		current_humi_addr = HUMIDITY_START_ADDR;

	return HAL_OK;
}

static HAL_StatusTypeDef Take_Measurements(uint16_t* phot, uint16_t* temp, uint16_t* press, uint16_t* humi)
{
	uint8_t dht_vals[4] = {0};

	if (DHT11_Read(dht_vals) != HAL_OK)
		return HAL_ERROR;

	if (HAL_ADC_Start(&hadc1) != HAL_OK)
		return HAL_ERROR;

	if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK)
		return HAL_ERROR;

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
  /* USER CODE BEGIN 2 */
  //Initialize ADC for photoresistor reading
  HAL_ADCEx_Calibration_Start(&hadc1);

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
  HAL_Delay(1000);
  while (1)
  {
	  uint16_t phot_to_write = 0, temp_to_write = 0;
	  uint16_t pressure_to_write = 0, humidity_to_write = 0;

	  if (Take_Measurements(&phot_to_write, &temp_to_write, &pressure_to_write, &humidity_to_write) != HAL_OK)
		  Error_Handler();

	  if (Write_Measured_Data(phot_to_write, temp_to_write, pressure_to_write, humidity_to_write) != HAL_OK)
		  Error_Handler();

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
