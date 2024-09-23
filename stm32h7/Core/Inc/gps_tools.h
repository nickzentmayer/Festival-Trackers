/*
 * gps_tools.h
 *
 *  Created on: Sep 3, 2024
 *      Author: Tyler
 */

#ifndef GPS_TOOLS
#define GPS_TOOLS

#include "stm32h7xx_hal.h" // Include the HAL library for your STM32 series
#include <stdbool.h>

// Define a struct to hold GPS data
typedef struct {
    bool valid;
    float latitude;
    float longitude;
    int hours;
    int minutes;
    float seconds;
    int num_satellites;
} GPS_Data;


// Function prototypes
bool Process_GGA_Sentence(const char *gga_sentence, GPS_Data *gps_data);
float convert_to_decimal(float nmea_coordinate, char direction);

#endif // GPS_TOOLS
