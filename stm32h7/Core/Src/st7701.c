/*
 * st7701.cpp
 *
 *  Created on: Oct 18, 2024
 *      Author: Tyler
 */

#include <st7701.h>
#include "stm32h7xx_hal.h"

#define ST7701_CMD 0x00 // C/D bit = 0 for command
#define ST7701_DATA 0x01 // C/D bit = 1 for data

// Define GPIO pins (Adjust as per your connections)
#define ST7701_MOSI_PIN GPIO_PIN_7
#define ST7701_MOSI_PORT GPIOA
#define ST7701_SCK_PIN GPIO_PIN_3
#define ST7701_SCK_PORT GPIOB
#define ST7701_CS_PIN GPIO_PIN_14
#define ST7701_CS_PORT GPIOA

// Function to set or reset GPIO pins (bit-banging)
static void ST7701_Delay(void) {
    // Small delay to simulate the clock period (adjust as needed)
    for (volatile int i = 0; i < 100; i++);
}

static void ST7701_SetCS(uint8_t state) {
    HAL_GPIO_WritePin(ST7701_CS_PORT, ST7701_CS_PIN, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

static void ST7701_SetSCK(uint8_t state) {
    HAL_GPIO_WritePin(ST7701_SCK_PORT, ST7701_SCK_PIN, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

static void ST7701_SetMOSI(uint8_t state) {
    HAL_GPIO_WritePin(ST7701_MOSI_PORT, ST7701_MOSI_PIN, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

static void ST7701_Send9Bit(uint16_t data) {
    // Pull CS low to begin transmission
    ST7701_SetCS(0);

    // Send 9-bit data, MSB first
    for (int i = 8; i >= 0; i--) {
        // Set MOSI to the current bit
        ST7701_SetMOSI((data >> i) & 0x01);

        // Toggle clock
        ST7701_SetSCK(1);
        ST7701_Delay();
        ST7701_SetSCK(0);
        ST7701_Delay();
    }

    // Pull CS high to end transmission
    ST7701_SetCS(1);
}

HAL_StatusTypeDef ST7701_SendCommand(uint8_t cmd) {
    // Send command with C/D bit = 0 (Command mode)
    uint16_t command = (ST7701_CMD << 8) | cmd;
    ST7701_Send9Bit(command);
    return HAL_OK;
}

HAL_StatusTypeDef ST7701_SendData(uint8_t data) {
    // Send data with C/D bit = 1 (Data mode)
    uint16_t dataWithCD = (ST7701_DATA << 8) | data;
    ST7701_Send9Bit(dataWithCD);
    return HAL_OK;
}


HAL_StatusTypeDef ST7701_Init(void) {
    HAL_StatusTypeDef status;

    // Reset the display using PE10
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET); // Assert reset (PE10 low)
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); // Release reset (PE10 high)
    HAL_Delay(50);

    // Send initialization commands
    status = ST7701_SendCommand(0x01); // Software reset
    if (status != HAL_OK) return status;
    HAL_Delay(20); // Wait for reset to complete

    status = ST7701_SendCommand(0x11); // Sleep Out
    if (status != HAL_OK) return status;
    HAL_Delay(20); // Wait for sleep out

    status = ST7701_SendCommand(0x29); // Display ON
    if (status != HAL_OK) return status;

    return HAL_OK;
}

