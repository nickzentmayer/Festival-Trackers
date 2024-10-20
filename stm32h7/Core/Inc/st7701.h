/*
 * st7701.h
 *
 *  Created on: Oct 18, 2024
 *      Author: Tyler
 */

#ifndef SRC_ST7701_H_
#define SRC_ST7701_H_

#include "stm32h7xx_hal.h"

// Function to initialize the ST7701 display
HAL_StatusTypeDef ST7701_Init(void);

// Function to send a command to the ST7701 using bit-banging
HAL_StatusTypeDef ST7701_SendCommand(uint8_t cmd);

// Function to send data to the ST7701 using bit-banging
HAL_StatusTypeDef ST7701_SendData(uint8_t data);

#endif /* SRC_ST7701_H_ */
