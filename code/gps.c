/**
  ******************************************************************************
  * @file           : gps.c
  * @brief          : mini library for L76-M33 GPS receiver using minmea library
  ******************************************************************************
  * @attention
  * Author: MTFinding
  ******************************************************************************
*/

#include <main.h>
#include <gps.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "minmea.h"

char GPS_recBuffer[GPS_receiveLen];
uint16_t GPS_bufPoint = 0;

_Bool get_gnrmc_line(char *input, char *output, int outputSize) {
    // Check for NULL input or output
    if (input == NULL || output == NULL) {
        return 0; // Return false
    }

    char *line = strtok(input, "\r\n");
    while (line != NULL) {
        if (strncmp(line, "$GNRMC", 6) == 0) {
            // Found a line starting with $GNRMC
            int len = strlen(line);
            if (len >= outputSize) {
                return 0; // Not enough space in the output buffer
            }

            strncpy(output, line, len);
            output[len] = '\0'; // Add null terminator
            return 1; // Return true
        }

        line = strtok(NULL, "\r\n");
    }

    return 0; // No $GNRMC line found
}

_Bool GPS_process(char *line, float *latitude, float *longitude, uint8_t *numSatellites) {
//	if(line == NULL) return false;

	switch (minmea_sentence_id(line, false)) {
	case MINMEA_SENTENCE_RMC: {
		struct minmea_sentence_rmc frame;
		if (minmea_parse_rmc(&frame, line)) {
//                minmea_rescale(&frame.latitude, 1000);
//                minmea_rescale(&frame.longitude, 1000);
			*latitude = minmea_tocoord(&frame.latitude);
			*longitude =  minmea_tocoord(&frame.longitude);
		}
		else {
			return false;
		}
	} break;

	case MINMEA_INVALID: {
		return false;
	} break;

	default: {
		return false;
	} break;
	}

   return true;
}
