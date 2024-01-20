/*
 * ADXL345.h
 *
 *  Created on: Nov 25, 2023
 *      Author: fevzi
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Private defines ------------------------------------------------------------*/

#define ADXL345_DEV_ADDR	0x53<<1
#define ADXL345_I2C 		hi2c2

extern uint8_t ADXL345_chipid;
extern uint8_t ADXL345_data_rec[6];

/* Function Prototypes ------------------------------------------------------------*/

void ADXL345_Write(uint8_t reg, uint8_t value);
void ADXL345_Read_Values(uint8_t reg);
void ADXL345_Read_Address(uint8_t reg);
void ADXL345_Init(void);


#endif /* INC_ADXL345_H_ */
