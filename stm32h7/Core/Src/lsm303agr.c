/*
 * lsm303agr.c
 *
 *  Created on: Sep 3, 2024
 *      Author: Tyler
 */

#include "lsm303agr.h"

HAL_StatusTypeDef LSM303AGR_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    // set accelerometer to 50Hz, normal power mode, all axes enabled
    data = 0x57; // 01010111
    if (HAL_I2C_Mem_Write(hi2c, ADDRESS_ACC, REG_CTRL_REG1_A, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    // set magnetometer to 50Hz, normal power mode, continuous read
    data = 0x88; // 10001000
    if (HAL_I2C_Mem_Write(hi2c, ADDRESS_ACC, CFG_REG_A_M, I2C_MEMADD_SIZE_8BIT, &data, 1, 100) != HAL_OK) {
            return HAL_ERROR;
        }
    return HAL_OK;
}

HAL_StatusTypeDef LSM303AGR_ReadAccel(I2C_HandleTypeDef *hi2c, LSM303AGR_AccelData *data) {
    uint8_t buffer[6];

    // the bitwise OR of 0x80 ensures the auto-increment bit in the register-to-read is set
    if (HAL_I2C_Mem_Read(hi2c, ADDRESS_ACC, REG_OUT_X_L_A | 0x80, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    data->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    data->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    data->z = (int16_t)(buffer[4] | (buffer[5] << 8));

    return HAL_OK;
}

HAL_StatusTypeDef LSM303AGR_ReadMag(I2C_HandleTypeDef *hi2c, LSM303AGR_MagData *data) {
    uint8_t buffer[6];

    if (HAL_I2C_Mem_Read(hi2c, ADDRESS_MAG, REG_OUT_X_L_M | 0x80, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100) != HAL_OK) {
        return HAL_ERROR;
    }

    data->x = (int16_t)(buffer[0] | (buffer[1] << 8));
    data->y = (int16_t)(buffer[2] | (buffer[3] << 8));
    data->z = (int16_t)(buffer[4] | (buffer[5] << 8));

    return HAL_OK;
}

HAL_StatusTypeDef LSM303AGR_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t regAddr, uint8_t *data, uint16_t size) {
    return HAL_I2C_Mem_Read(hi2c, ADDRESS_ACC, regAddr, I2C_MEMADD_SIZE_8BIT, data, size, 1000);
}

//HAL_StatusTypeDef LSM303AGR_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t deviceAddr, uint8_t regAddr, uint8_t *data, uint16_t size) {
//    return HAL_I2C_Mem_Write(hi2c, ADDRESS_MAG, regAddr, I2C_MEMADD_SIZE_8BIT, data, size, 1000);
//}
