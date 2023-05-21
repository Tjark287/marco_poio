/**
  ******************************************************************************
  * @file           : gps.c
  * @brief          : mini library for L76-M33 GPS receiver
  ******************************************************************************
  * @attention
  * Author: MTFinding
  ******************************************************************************
*/

#include <main.h>
#include <gps.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

char GPS_recBuffer[GPS_receiveLen];
uint16_t GPS_bufPoint = 0;

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

_Bool GPS_process(char *gpsData, double *latitude, double *longitude, uint8_t *numSatellites)
{
    if (gpsData == NULL)
    {
        return false;
    }

    char *gprmc = strstr(gpsData, "GPRMC");
    char *gnrmc = strstr(gpsData, "GNRMC");
    char *gpgga = strstr(gpsData, "GPGGA");
    char *gpgsa = strstr(gpsData, "GPGSA");

    bool valid = false;

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
