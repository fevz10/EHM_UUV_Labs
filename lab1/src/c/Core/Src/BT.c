/*
 * BT.c
 *
 *  Created on: Oct 30, 2023
 *      Author: fdereli
 */
#include "BT.h"
/* Private variables ---------------------------------------------------------*/

extern UART_HandleTypeDef BT_UART;
uint8_t BT_TxDataBuffer[BT_UART_TX_BUF_SIZE];
uint8_t BT_RxDataBuffer[BT_UART_RX_BUF_SIZE];
uint16_t BT_RxCharCounter = 0;




/* Private functions ---------------------------------------------------------*/

void extractIntegersFromCharArray(const char *input, int *intArray, int arraySize)
{
    int i = 0;
    int count = 0;

    while (input[i] != '\0' && count < arraySize) {
        // Skip non-digit characters
        while (input[i] != '\0' && !isdigit(input[i])) {
            i++;
        }

        // Read digits and build the integer value
        int value = 0;
        while (input[i] != '\0' && isdigit(input[i])) {
            value = value * 10 + (input[i] - '0');
            i++;
        }

        // Store the extracted value in the array
        intArray[count] = value;
        count++;
    }
}

void BT_RxData(void)
{
	uint8_t rxChar;
	uint32_t tmp_state = 0;

	tmp_state = BT_UART.gState;
	if((tmp_state == HAL_UART_STATE_BUSY_RX) || (tmp_state != HAL_UART_STATE_BUSY_TX_RX))
	{
		rxChar = (uint8_t)(BT_UART.Instance->DR & (uint8_t)0x00FF);
		BT_RxDataBuffer[BT_RxCharCounter] = rxChar;
		BT_RxCharCounter++;
	}

	if(BT_RxCharCounter >= BT_UART_RX_BUF_SIZE)
	{
		BT_RxCharCounter = 0;
	}

	if((BT_RxDataBuffer[BT_RxCharCounter-2] == 13) && (BT_RxDataBuffer[BT_RxCharCounter-1] == 10)) // if end of the received data is \r\n?
	{
		int pwmValues[2] = {0};

		uint8_t receivedData[BT_RxCharCounter];
		for(uint16_t i=0; i<BT_RxCharCounter; i++)
		{
			receivedData[i] = BT_RxDataBuffer[i];
		}
		memset(BT_RxDataBuffer, 0x00, BT_UART_RX_BUF_SIZE*sizeof(uint8_t));
		HAL_UART_Transmit(&BT_UART, receivedData, BT_RxCharCounter, HAL_MAX_DELAY);
		BT_RxCharCounter = 0;

		if(strstr((const char*)receivedData, (const char*)"MONE_") != NULL)
		{
			extractIntegersFromCharArray(receivedData, pwmValues, 2);
			TIM4->CCR1 = pwmValues[0];
			TIM4->CCR2 = pwmValues[1];
		}
		if(strstr((const char*)receivedData, (const char*)"MTWO_") != NULL)
		{
			extractIntegersFromCharArray(receivedData, pwmValues, 2);
			TIM4->CCR3 = pwmValues[0];
			TIM4->CCR4 = pwmValues[1];
		}


		// Added the motor states given below.
		/*
		if(strstr((const char*)receivedData, (const char*)"MOTOR1-50-20n") != NULL)		// if received message includes MOTOR1ON\r\n string array?
		{
			HAL_GPIO_WritePin(Motor1_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_SET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR2ON\r\n") != NULL)		// if received message includes MOTOR2ON\r\n string array?
		{
			HAL_GPIO_WritePin(Motor2_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_SET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR3ON\r\n") != NULL)		// if received message includes MOTOR3ON\r\n string array?
		{
			HAL_GPIO_WritePin(Motor3_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_SET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR4ON\r\n") != NULL)		// if received message includes MOTOR4ON\r\n string array?
		{
			HAL_GPIO_WritePin(Motor4_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_SET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR1OFF\r\n") != NULL)		// if received message includes MOTOR1OFF\r\n string array?
		{
			HAL_GPIO_WritePin(Motor1_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_RESET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR2OFF\r\n") != NULL)		// if received message includes MOTOR2OFF\r\n string array?
		{
			HAL_GPIO_WritePin(Motor2_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_RESET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR3OFF\r\n") != NULL)		// if received message includes MOTOR3OFF\r\n string array?
		{
			HAL_GPIO_WritePin(Motor3_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_RESET);
		}
		if(strstr((const char*)receivedData, (const char*)"MOTOR4OFF\r\n") != NULL)		// if received message includes MOTOR1ON\r\n string array?
		{
			HAL_GPIO_WritePin(Motor4_Enable_GPIO_Port, Motor1_Enable_Pin, GPIO_PIN_RESET);
		}
		*/
	}


}
