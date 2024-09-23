/*
 * max17048.c
 *
 *  Created on: Sep 5, 2024
 *      Author: Tyler
 */

#include "max17048.h"

HAL_StatusTypeDef MAX17048_Read_Battery(I2C_HandleTypeDef *hi2c, MAX17048_BatteryData *data) {
    uint8_t buffer[2];
    int16_t signed_combined;  // Use signed 16-bit for CRATE
    uint16_t combined;

    // Read SOC register
    if (HAL_I2C_Mem_Read(hi2c, ADDRESS_FG, REG_SOC, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    combined = (buffer[0] << 8) | buffer[1];
    data->soc = (float)combined / 256.0f;

    // Read charge rate register (CRATE)
    if (HAL_I2C_Mem_Read(hi2c, ADDRESS_FG, REG_CRATE, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine the bytes and interpret as a signed 16-bit integer
    signed_combined = (int16_t)((buffer[0] << 8) | buffer[1]);

    // Apply scaling factor
    data->chg_rate = (float)signed_combined * 0.208f;

    return HAL_OK;
}

