/*
 * dht11.c
 *
 *  Created on: Sep 4, 2024
 *      Author: arima
 */

#include "dht11.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

static void US_Delay(const uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < time);
}

static void DHT11_Send_Start(void)
{
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET);
	US_Delay(18000);
	HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);
	US_Delay(40);
}

static uint8_t DHT11_Read_Byte(void)
{
	uint8_t read_data = 0;

	for (int i = 0; i < 8; i++) {
		while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_RESET);

		__HAL_TIM_SET_COUNTER(&htim2, 0);
		while (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_SET);

		if (__HAL_TIM_GET_COUNTER(&htim2) > 50) {
			read_data |= 1 << (7 - i);
		}
	}

	return read_data;
}

HAL_StatusTypeDef DHT11_Read(uint8_t* values)
{
	uint8_t check_sum = 0;

	DHT11_Send_Start();

	if (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_RESET) {
		US_Delay(80);
		if (HAL_GPIO_ReadPin(DHT11_GPIO_Port, DHT11_Pin) == GPIO_PIN_SET) {
			US_Delay(80);
			for (uint8_t i = 0; i < 4; i++) {
				values[i] = DHT11_Read_Byte();
			}
			check_sum = DHT11_Read_Byte();
		}
	} else {
		return HAL_ERROR;
	}

	if (values[0] + values[1] + values[2] + values[3] == check_sum) {
		return HAL_OK;
	} else {
		return HAL_ERROR;
	}
}


















