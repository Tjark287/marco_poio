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

/*
_Bool GPS_process(char *gpsData, float *latitude, float *longitude, uint8_t *numSatellites) {

	// Check sentence validity
	if (!minmea_check(gpsData, false)) {
		return 0;
	}

	// Determine sentence type
	enum minmea_sentence_id sentence_id = minmea_sentence_id(gpsData, false);
	switch (sentence_id) {

		case MINMEA_SENTENCE_RMC: {
			struct minmea_sentence_rmc frame;
			if (minmea_parse_rmc(&frame, gpsData)) {
				// Process RMC sentence
				// Access frame data using the structure members
				*latitude = minmea_tocoord(&frame.latitude);
				*longitude = minmea_tocoord(&frame.longitude);
				// Return false if zero-data
				if(latitude == 0 || longitude == 0) return 0;
				return 1;
			}
		} break;


//		case MINMEA_SENTENCE_GGA: {
//			struct minmea_sentence_gga frame;
//			if (minmea_parse_gga(&frame, gpsData)) {
//				// Process GGA sentence
//				// Access frame data using the structure members
//				*latitude = minmea_tocoord(&frame.latitude);
//				*longitude = minmea_tocoord(&frame.longitude);
//				*numSatellites = frame.satellites_tracked;
//				return 1;
//			}
//		} break;

		// Add more cases for other sentence types as needed

		default:
			return false;
			break;
	}
	return 0;
}
*/
/*
_Bool GPS_process(char *gpsData, float *latitude, float *longitude, uint8_t *numSatellites)
{
    if (gpsData == NULL)
    {
        return false;
    }

    char *gprmc = strstr(gpsData, "GPRMC");
    char *gnrmc = strstr(gpsData, "GNRMC");
    char *gpgga = strstr(gpsData, "GPGGA");
    char *gpgsa = strstr(gpsData, "GPGSA");

    _Bool valid = false;

    // Parse GPRMC or GNRMC
    if (gprmc != NULL || gnrmc != NULL)
    {
        char *token;
        int commaCount = 0;

        // Use GPRMC or GNRMC sentence depending on which is present
        char *rmc = gprmc ? gprmc : gnrmc;

        token = strtok(rmc, ",");

        while (token != NULL)
        {
            commaCount++;

            if (commaCount == 4 && latitude != NULL)
            {
                *latitude = atof(token);
            }

            if (commaCount == 6 && longitude != NULL)
            {
                *longitude = atof(token);
            }

            if (commaCount == 3) // Status field
            {
                if (strcmp(token, "A") == 0)
                {
                    valid = true;
                }
            }

            token = strtok(NULL, ",");
        }
    }

    // Parse GPGGA
    if (gpgga != NULL)
    {
        char *token;
        int commaCount = 0;

        token = strtok(gpgga, ",");

        while (token != NULL)
        {
            commaCount++;

            if (commaCount == 8 && numSatellites != NULL)
            {
                *numSatellites = atoi(token);
            }

            token = strtok(NULL, ",");
        }
    }

    // Parse GPGSA
    if (gpgsa != NULL)
    {
        char *token;
        int commaCount = 0;

        token = strtok(gpgsa, ",");

        while (token != NULL)
        {
            commaCount++;

            // Checking mode and fix type
            if ((commaCount == 2 && strcmp(token, "M") == 0) || (commaCount == 3 && strcmp(token, "1") == 0))
            {
                valid = false;
            }

            token = strtok(NULL, ",");
        }
    }

    return valid;
}
*/
