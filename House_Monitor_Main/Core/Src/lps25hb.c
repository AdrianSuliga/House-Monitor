/*
 * lps25hb.c
 *
 *  Created on: Sep 4, 2024
 *      Author: arima
 */

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

#define TIMEOUT 100

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





