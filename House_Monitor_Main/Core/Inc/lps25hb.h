/*
 * lps25hb.h
 *
 *  Created on: Sep 4, 2024
 *      Author: arima
 */

#ifndef INC_LPS25HB_H_
#define INC_LPS25HB_H_

#include <stm32f1xx.h>

// Initialize LPS25HB sensor, power it on
// and set pressure and temperature reading
// frequency to 25 Hz
HAL_StatusTypeDef LPS_Init(void);

// Calibrate LPS25HB sensor if its readings
// doesn't much actual values
void LPS_Calibrate(uint16_t value);

// Read temperature from TEMP_OUT_L and
// TEMP_OUT_H registers
float LPS_Read_Temp(void);

// Read pressure from PRESS_OUT_XL,
// PRESS_OUT_L and PRESS_OUT_H registers
float LPS_Read_Pressure(void);


#endif /* INC_LPS25HB_H_ */
