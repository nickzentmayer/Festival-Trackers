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
//    HAL_StatusTypeDef status;

    // Reset the display using PE10
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET); // Assert reset (PE10 low)
    HAL_Delay(5);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET); // Release reset (PE10 high)
    HAL_Delay(50);

    // Send initialization commands
	//#if 1 //zheng shao
	ST7701_SendCommand(0xFF);
	ST7701_SendData(0x77);
	ST7701_SendData(0x01);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x10);

	ST7701_SendCommand(0xC0);
	ST7701_SendData(0x3B);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xC1);
	ST7701_SendData(0x0B);	//VBP
	ST7701_SendData(0x02);

	ST7701_SendCommand(0xC2);
	ST7701_SendData(0x00);
	ST7701_SendData(0x02);

	ST7701_SendCommand(0xCC);
	ST7701_SendData(0x10);

	ST7701_SendCommand(0xCD);
	ST7701_SendData(0x08);



	ST7701_SendCommand ( 0xB0); //Positive Voltage Gamma Control
	ST7701_SendData ( 0x02);
	ST7701_SendData ( 0x13);
	ST7701_SendData ( 0x1B);
	ST7701_SendData ( 0x0D);
	ST7701_SendData ( 0x10);
	ST7701_SendData ( 0x05);
	ST7701_SendData ( 0x08);
	ST7701_SendData ( 0x07);
	ST7701_SendData ( 0x07);
	ST7701_SendData ( 0x24);
	ST7701_SendData ( 0x04);
	ST7701_SendData ( 0x11);
	ST7701_SendData ( 0x0E);
	ST7701_SendData ( 0x2C);
	ST7701_SendData ( 0x33);
	ST7701_SendData ( 0x1D);

	ST7701_SendCommand ( 0xB1); //Negative Voltage Gamma Control
	ST7701_SendData ( 0x05);
	ST7701_SendData ( 0x13);
	ST7701_SendData ( 0x1B);
	ST7701_SendData ( 0x0D);
	ST7701_SendData ( 0x11);
	ST7701_SendData ( 0x05);
	ST7701_SendData ( 0x08);
	ST7701_SendData ( 0x07);
	ST7701_SendData ( 0x07);
	ST7701_SendData ( 0x24);
	ST7701_SendData ( 0x04);
	ST7701_SendData ( 0x11);
	ST7701_SendData ( 0x0E);
	ST7701_SendData ( 0x2C);
	ST7701_SendData ( 0x33);
	ST7701_SendData ( 0x1D);

	ST7701_SendCommand(0xFF);
	ST7701_SendData(0x77);
	ST7701_SendData(0x01);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x11);

	ST7701_SendCommand(0xB0);
	ST7701_SendData(0x5d);//5d

	ST7701_SendCommand(0xB1); 	//VCOM amplitude setting
	ST7701_SendData(0x43); //43

	ST7701_SendCommand(0xB2); 	//VGH Voltage setting
	ST7701_SendData(0x81);	//12V

	ST7701_SendCommand(0xB3);
	ST7701_SendData(0x80);

	ST7701_SendCommand(0xB5); 	//VGL Voltage setting
	ST7701_SendData(0x43);	//-8.3V

	ST7701_SendCommand(0xB7);
	ST7701_SendData(0x85);

	ST7701_SendCommand(0xB8);
	ST7701_SendData(0x20);

	ST7701_SendCommand(0xC1);
	ST7701_SendData(0x78);

	ST7701_SendCommand(0xC2);
	ST7701_SendData(0x78);

	ST7701_SendCommand(0xD0);
	ST7701_SendData(0x88);

	ST7701_SendCommand(0xE0);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x02);

	ST7701_SendCommand(0xE1);
	ST7701_SendData(0x03);
	ST7701_SendData(0xA0);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x04);
	ST7701_SendData(0xA0);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x20);
	ST7701_SendData(0x20);

	ST7701_SendCommand(0xE2);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xE3);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x11);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xE4);
	ST7701_SendData(0x22);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xE5);
	ST7701_SendData(0x05);
	ST7701_SendData(0xEC);
	ST7701_SendData(0xA0);
	ST7701_SendData(0xA0);
	ST7701_SendData(0x07);
	ST7701_SendData(0xEE);
	ST7701_SendData(0xA0);
	ST7701_SendData(0xA0);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xE6);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x11);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xE7);
	ST7701_SendData(0x22);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xE8);
	ST7701_SendData(0x06);
	ST7701_SendData(0xED);
	ST7701_SendData(0xA0);
	ST7701_SendData(0xA0);
	ST7701_SendData(0x08);
	ST7701_SendData(0xEF);
	ST7701_SendData(0xA0);
	ST7701_SendData(0xA0);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xEB);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x40);
	ST7701_SendData(0x40);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0xED);
	ST7701_SendData(0xFF);
	ST7701_SendData(0xFF);
	ST7701_SendData(0xFF);
	ST7701_SendData(0xBA);
	ST7701_SendData(0x0A);
	ST7701_SendData(0xBF);
	ST7701_SendData(0x45);
	ST7701_SendData(0xFF);
	ST7701_SendData(0xFF);
	ST7701_SendData(0x54);
	ST7701_SendData(0xFB);
	ST7701_SendData(0xA0);
	ST7701_SendData(0xAB);
	ST7701_SendData(0xFF);
	ST7701_SendData(0xFF);
	ST7701_SendData(0xFF);

	ST7701_SendCommand(0xEF);
	ST7701_SendData(0x10);
	ST7701_SendData(0x0D);
	ST7701_SendData(0x04);
	ST7701_SendData(0x08);
	ST7701_SendData(0x3F);
	ST7701_SendData(0x1F);

	ST7701_SendCommand(0xFF);
	ST7701_SendData(0x77);
	ST7701_SendData(0x01);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x13);

	ST7701_SendCommand(0xEF);
	ST7701_SendData(0x08);

	ST7701_SendCommand(0xFF);
	ST7701_SendData(0x77);
	ST7701_SendData(0x01);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	ST7701_SendData(0x00);
	//
	//#if 0
	//WriteComm (0xFF);
	//WriteData (0x77);
	//WriteData (0x01);
	//WriteData (0x00);
	//WriteData (0x00);
	//WriteData (0x12);
	//WriteComm (0xD1);
	//WriteData (0x81);
	//WriteData (0x08);
	//WriteData (0x03);
	//WriteData (0x20);
	//WriteData (0x08);
	//WriteData (0x01);
	//WriteData (0xA0);
	//WriteData (0x01);
	//WriteData (0xE0);
	//WriteData (0xA0);
	//WriteData (0x01);
	//WriteData (0xE0);
	//WriteData (0x03);
	//WriteData (0x20);
	//WriteComm (0xD2);
	//WriteData (0x08);
	//#endif
	/////////////////Bring up the internal test picture///////////////////////////////////


	ST7701_SendCommand(0x11);

	HAL_Delay(120);

	ST7701_SendCommand(0x29);

	ST7701_SendCommand(0x36);
	ST7701_SendData(0x00);

	ST7701_SendCommand(0x3A);
	ST7701_SendData(0x60);//0x60 18bit   0x50 16bit
	//#endif

    return HAL_OK;
}

