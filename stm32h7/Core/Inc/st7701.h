/*
 * st7701.h
 *
 *  Created on: Oct 18, 2024
 *      Author: Tyler
 */

#ifndef SRC_ST7701_H_
#define SRC_ST7701_H_

#include "stm32h7xx_hal.h"

HAL_StatusTypeDef ST7701_SendCommand(SPI_HandleTypeDef *hspi, uint8_t cmd);
HAL_StatusTypeDef ST7701_SendData(SPI_HandleTypeDef *hspi, uint8_t data);
HAL_StatusTypeDef ST7701_Init(SPI_HandleTypeDef *hspi);

#endif /* SRC_ST7701_H_ */
