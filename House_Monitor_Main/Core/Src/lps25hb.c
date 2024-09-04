/*
 * lps25hb.c
 *
 *  Created on: Sep 4, 2024
 *      Author: arima
 */

#include <math.h>
#include "lps25hb.h"
#include "i2c.h"

#define LPS_ADDR 0xBA

#define REF_P_XL 0x08
#define REF_P_L 0x09
#define REF_P_H 0x0A
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define PRESS_OUT_XL 0x28
#define PRESS_OUT_L 0x29
#define PRESS_OUT_H 0x2A
#define TEMP_OUT_L 0x2B
#define TEMP_OUT_H 0x2C
#define RPDS_L 0x39
#define RPDS_H 0x3A

#define TIMEOUT 100
#define HEIGHT_OVER_SEE_LEVEL 247
#define PRESSURE_CONSTANT 0.034162608734308

static uint8_t LPS_Read_Register(uint8_t reg)
{
	uint8_t val = 0;
	HAL_I2C_Mem_Read(&hi2c1, LPS_ADDR, reg, 1, &val, sizeof(val), TIMEOUT);
	return val;
}

static void LPS_Write_Register(uint8_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(&hi2c1, LPS_ADDR, reg, 1, &value, sizeof(value), TIMEOUT);
}

HAL_StatusTypeDef LPS_Init(void)
{
	if (LPS_Read_Register(WHO_AM_I) != 0xBD)
		return HAL_ERROR;

	LPS_Write_Register(CTRL_REG1, 0xC0);
	return HAL_OK;
}

void LPS_Calibrate(uint16_t value)
{
	LPS_Write_Register(RPDS_L, value);
	LPS_Write_Register(RPDS_H, value >> 8);
}

float LPS_Read_Temp(void)
{
	int16_t temp = 0;

	if (HAL_I2C_Mem_Read(&hi2c1, LPS_ADDR, TEMP_OUT_L | 0x80, 1, (uint8_t*)&temp, 2, TIMEOUT) != HAL_OK)
		Error_Handler();

	return 42.5f + temp / 480.0f;
}

float LPS_Read_Pressure(void)
{
	int32_t value = 0;

	if (HAL_I2C_Mem_Read(&hi2c1, LPS_ADDR, PRESS_OUT_XL | 0x80, 1, (uint8_t*)&value, 3, TIMEOUT) != HAL_OK)
		Error_Handler();

	float read_pressure = value / 4096.0f;
	float temp = LPS_Read_Temp() + 273.15f;
	float actual_pressure = read_pressure * exp(PRESSURE_CONSTANT * HEIGHT_OVER_SEE_LEVEL / temp);
	return actual_pressure;
}














