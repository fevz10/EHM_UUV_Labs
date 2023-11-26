/*
 * MC5883L.h
 *
 *  Created on: Nov 25, 2023
 *      Author: fevzi
 */

#ifndef INC_MC5883L_H_
#define INC_MC5883L_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Private defines ------------------------------------------------------------*/

#define MC5883L_DEV_ADDR	0x0D<<1
#define MC5883L_I2C 		hi2c2

extern uint8_t MC5883L_chipid;
extern uint8_t MC5883L_data_rec[6];

/* Function Prototypes ------------------------------------------------------------*/

void MC5883L_Write(uint8_t reg, uint8_t value);
void MC5883L_Read_Values(uint8_t reg);
void MC5883L_Read_Address(uint8_t reg);
void MC5883L_Init(void);




#endif /* INC_MC5883L_H_ */
