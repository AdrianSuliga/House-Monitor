/*
 * dht11.h
 *
 *  Created on: Sep 4, 2024
 *      Author: arima
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include <stm32f1xx.h>

// Read values from DHT11 sensor
// This function handles initializing
// sensor by sending it START signal
// values - array of four bytes that
// this function writes to
// return - HAL_OK or HAL_ERROR
HAL_StatusTypeDef DHT11_Read(uint8_t* values);

#endif /* INC_DHT11_H_ */
