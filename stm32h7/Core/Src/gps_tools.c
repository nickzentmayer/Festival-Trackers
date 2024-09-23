/*
 * gps_tools.c
 *
 *  Created on: Sep 3, 2024
 *      Author: Tyler
 */

#include "gps_tools.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

bool Process_GGA_Sentence(const char *gga_sentence, GPS_Data *gps_data) {
	gps_data->valid = false;

    // Check if it is a GGA sentence
    if (strncmp(gga_sentence, "$GPGGA", 6) != 0) {
        return false; // Not a GGA sentence
    }

    // Find the end of the sentence (length of checksum is always 2 characters + '*')
	const char *end_of_sentence = gga_sentence + strlen(gga_sentence) - 3;

	// Extract and calculate checksum
	uint8_t calculated_checksum = 0;
	for (const char *p = gga_sentence + 1; p < end_of_sentence; p++) {
		calculated_checksum ^= *p;
	}

    // Convert the provided checksum to an integer
    uint8_t provided_checksum = (uint8_t)strtol(end_of_sentence + 1, NULL, 16);

    // Validate the checksum
    if (calculated_checksum != provided_checksum) {
        return false; // Checksum mismatch
    }

    // Process the GGA fields
    char *token;
    char *nmea_copy = strdup(gga_sentence);
    token = strtok(nmea_copy, ",");

    int field_number = 0;
    float raw_latitude = 0.0;
    float raw_longitude = 0.0;
    char lat_dir = 'N';
    char lon_dir = 'E';

    while (token != NULL) {
        switch (field_number) {
            case 1: // Timestamp
                if (*token) {
                    sscanf(token, "%2d%2d%f", &gps_data->hours, &gps_data->minutes, &gps_data->seconds);
                }
                break;
            case 2: // Latitude
                if (*token) {
                    raw_latitude = atof(token);
                }
                break;
            case 3: // Latitude direction (N/S)
                if (*token) {
                    lat_dir = *token;
                }
                break;
            case 4: // Longitude
                if (*token) {
                    raw_longitude = atof(token);
                }
                break;
            case 5: // Longitude direction (E/W)
                if (*token) {
                    lon_dir = *token;
                }
                break;
            case 6: // Fix status
                gps_data->valid = (*token != '0');
                break;
            case 7: // Number of satellites
				gps_data->num_satellites = atoi(token);
				break;
        }
        token = strtok(NULL, ",");
        field_number++;
    }

    free(nmea_copy);

    // Convert to decimal format if the sentence is valid
    if (gps_data->valid) {
        gps_data->latitude = convert_to_decimal(raw_latitude, lat_dir);
        gps_data->longitude = convert_to_decimal(raw_longitude, lon_dir);
    }

    return gps_data->valid;
}

float convert_to_decimal(float nmea_coordinate, char direction) {
    int degrees = (int)(nmea_coordinate / 100);
    float minutes = nmea_coordinate - (degrees * 100);
    float decimal = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W') {
        decimal *= -1;
    }

    return decimal;
}

