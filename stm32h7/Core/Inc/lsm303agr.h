/*
 * lsm303agr.h
 *
 *  Created on: Sep 3, 2024
 *      Author: Tyler
 */
#ifndef LSM303AGR_H
#define LSM303AGR_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define ADDRESS_ACC       0x19 << 1
#define ADDRESS_MAG       0x1E << 1

#define REG_WHO_AM_I_A    0x0F
#define REG_CTRL_REG1_A   0x20
#define CFG_REG_A_M       0x60
#define REG_OUT_X_L_A     0x28
#define REG_OUT_X_L_M     0x68

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} LSM303AGR_AccelData;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} LSM303AGR_MagData;

HAL_StatusTypeDef LSM303AGR_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LSM303AGR_ReadAccel(I2C_HandleTypeDef *hi2c, LSM303AGR_AccelData *data);
HAL_StatusTypeDef LSM303AGR_ReadMag(I2C_HandleTypeDef *hi2c, LSM303AGR_MagData *data);
HAL_StatusTypeDef LSM303AGR_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t regAddr, uint8_t *data, uint16_t size);
//HAL_StatusTypeDef LSM303AGR_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t regAddr, uint8_t *data, uint16_t size);

#endif
