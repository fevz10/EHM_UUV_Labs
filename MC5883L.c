/*
 * MC5883L.c
 *
 *  Created on: Nov 25, 2023
 *      Author: fevzi
 */
#include "MC5883L.h"

/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef MC5883L_I2C;


/* Private functions ---------------------------------------------------------*/

void MC5883L_Write(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&MC5883L_I2C, MC5883L_DEV_ADDR, data, 2, 100);
}

void MC5883L_Read_Values(uint8_t reg)
{
	HAL_I2C_Mem_Read(&MC5883L_I2C, MC5883L_DEV_ADDR, reg, 1, (uint8_t *)MC5883L_data_rec, 6, 100);
}

void MC5883L_Read_Address(uint8_t reg)
{
	HAL_I2C_Mem_Read(&MC5883L_I2C, MC5883L_DEV_ADDR, reg, 1, &MC5883L_chipid, 1, 100);
}

void MC5883L_Init(void)
{
	MC5883L_Read_Address(0x00); // read the DEVID

	MC5883L_Write(0x0B, 0x01);  // Define Set/Reset period
	MC5883L_Write(0x09, 0x1D);  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
}
