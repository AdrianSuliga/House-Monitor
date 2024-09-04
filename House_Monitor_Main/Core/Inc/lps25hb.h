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


#endif /* INC_LPS25HB_H_ */
