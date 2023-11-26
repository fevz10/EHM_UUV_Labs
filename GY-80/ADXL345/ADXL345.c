/*
 * ADXL345.c
 *
 *  Created on: Nov 25, 2023
 *      Author: fevzi
 */
#include "ADXL345.h"

/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef ADXL345_I2C;

/* Private functions ---------------------------------------------------------*/

void ADXL345_Write(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit(&ADXL345_I2C, ADXL345_DEV_ADDR, data, 2, 100);
}

void ADXL345_Read_Values(uint8_t reg)
{
	HAL_I2C_Mem_Read(&ADXL345_I2C, ADXL345_DEV_ADDR, reg, 1, (uint8_t *)ADXL345_data_rec, 6, 100);
}

void ADXL345_Read_Address(uint8_t reg)
{
	HAL_I2C_Mem_Read(&ADXL345_I2C, ADXL345_DEV_ADDR, reg, 1, &ADXL345_chipid, 1, 100);
}

void ADXL345_Init(void)
{
	ADXL345_Read_Address(0x00); // read the DEVID

	ADXL345_Write(0x31, 0x01);  // data_format range= +- 4g
	ADXL345_Write(0x2d, 0x00);  // reset all bits
	ADXL345_Write(0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
