/*
 * st7701.cpp
 *
 *  Created on: Oct 18, 2024
 *      Author: Tyler
 */

#include "st7701.h"

#define ST7701_CMD  0x00 // Command mode
#define ST7701_DATA 0x01 // Data mode

#define LCD_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET) // Select LCD
#define LCD_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET)   // Deselect LCD

HAL_StatusTypeDef ST7701_SendCommand(SPI_HandleTypeDef *hspi, uint8_t cmd) {
    // Send command mode (command/parameter bit = 0)
    uint8_t command[2] = {ST7701_CMD, cmd};
    return HAL_SPI_Transmit(hspi, command, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ST7701_SendData(SPI_HandleTypeDef *hspi, uint8_t data) {
    // Send data mode (command/parameter bit = 1)
    uint8_t dataArray[2] = {ST7701_DATA, data};
    return HAL_SPI_Transmit(hspi, dataArray, 2, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ST7701_Init(SPI_HandleTypeDef *hspi) {
    HAL_StatusTypeDef status;

    // Reset the display using PE10
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET); // Assert reset (PE10 low)
    HAL_Delay(20); // Wait for 20 ms
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); // Release reset (PE10 high)
    HAL_Delay(120); // Wait for 120 ms after reset

    // Select the LCD
    LCD_CS_LOW();

    // Send initialization commands
    status = ST7701_SendCommand(hspi, 0x01); // Software reset
    if (status != HAL_OK) return status;
    HAL_Delay(150); // Wait for reset to complete

    status = ST7701_SendCommand(hspi, 0x11); // Sleep Out
    if (status != HAL_OK) return status;
    HAL_Delay(120); // Wait for sleep out

    status = ST7701_SendCommand(hspi, 0x29); // Display ON
    if (status != HAL_OK) return status;

    // Deselect the LCD
    LCD_CS_HIGH();

    return HAL_OK;
}
