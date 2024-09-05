/*
 * eeprom.c
 *
 *  Created on: Sep 5, 2024
 *      Author: arima
 */

#include "i2c.h"

#define EEPROM_ADDR 0xA0
#define WRITE_TIMEOUT 6

static uint32_t last_write;

static void EEPROM_Wait(void)
{
	while (HAL_GetTick() - last_write < WRITE_TIMEOUT);
}

HAL_StatusTypeDef EEPROM_Write(uint32_t reg, const void* data, uint32_t size)
{
	HAL_StatusTypeDef rc;

	EEPROM_Wait();
	rc = HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, reg, 1, (void*)data, size, HAL_MAX_DELAY);
	last_write = HAL_GetTick();

	return rc;
}

HAL_StatusTypeDef EEPROM_Read(uint32_t reg, void* data, uint32_t size)
{
	EEPROM_Wait();
	return HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, reg, 1, data, size, HAL_MAX_DELAY);
}

HAL_StatusTypeDef EEPROM_Reset(uint8_t reset_value)
{
	HAL_StatusTypeDef status;

	for (uint16_t address = 0; address < 128; address++) {
		status = EEPROM_Write(address, (void*)&reset_value, sizeof(reset_value));

		if (status != HAL_OK)
			return status;
	}

	return HAL_OK;
}














