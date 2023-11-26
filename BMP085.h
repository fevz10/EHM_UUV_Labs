/*
 * BMP085.h
 *
 *  Created on: Nov 25, 2023
 *      Author: fevzi
 */

#ifndef INC_BMP085_H_
#define INC_BMP085_H_

#include "stm32f4xx_hal.h"
#include "main.h"


#define BMP085_DEV_ADDR		0x77 << 1
#define BMP085_I2C 			hi2c2

#define BMP085_ULTRALOWPOWER 0 //!< Ultra low power mode
#define BMP085_STANDARD 1      //!< Standard mode
#define BMP085_HIGHRES 2       //!< High-res mode
#define BMP085_ULTRAHIGHRES 3  //!< Ultra high-res mode

#define STD_ATM_PRESS 101325

// Sensor Init function
uint8_t BMP085_Init(uint8_t mode);
float BMP085_ReadTemperature(void);
int32_t BMP085_ReadPressure(void);

#endif /* INC_BMP085_H_ */
