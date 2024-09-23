/*
 * max17048.h
 *
 *  Created on: Sep 5, 2024
 *      Author: Tyler
 */



#ifndef INC_MAX17048_H_
#define INC_MAX17048_H_

#include "stm32h7xx_hal.h"

#define ADDRESS_FG  0x36 << 1 // 0x6C

#define REG_SOC		0x04
#define REG_MODE	0x06
#define REG_CRATE	0x16

typedef struct {
	float soc; // 1%/256
	float chg_rate; // .208%/hr
} MAX17048_BatteryData;

HAL_StatusTypeDef MAX17048_Read_Battery(I2C_HandleTypeDef *hi2c, MAX17048_BatteryData *data);

#endif /* INC_MAX17048_H_ */
