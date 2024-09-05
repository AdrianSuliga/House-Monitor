/*
 * eeprom.h
 *
 *  Created on: Sep 5, 2024
 *      Author: arima
 */

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include <stm32f1xx.h>

// Write size bits of data to EEPROM starting
// from address reg
HAL_StatusTypeDef EEPROM_Write(uint32_t reg, const void* data, uint32_t size);

// Read size bits of data from EEPROM starting
// from address reg
HAL_StatusTypeDef EEPROM_Read(uint32_t reg, void* data, uint32_t size);

// Set all the addresses in EEPROM memory to
// reset_value
HAL_StatusTypeDef EEPROM_Reset(uint8_t reset_value);

#endif /* INC_EEPROM_H_ */
