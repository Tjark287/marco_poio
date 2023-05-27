/**
  ******************************************************************************
  * @file           : helpHeidi.c
  * @brief          : Helper functions for MARCO POIo device
  ******************************************************************************
  * @attention
  * helping HEIDIs since 2023
  * Author: MTFinding
  ******************************************************************************
*/

#include <main.h>
#include <helpHeidi.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

void serialPrint(const char *str, ...)
{
    char buffer[256];
    va_list args;
    va_start(args, str);
    int len = vsnprintf(buffer, sizeof(buffer), str, args);
    va_end(args);

    // Add the newline character
    if (len > 0 && len < sizeof(buffer) - 1) {
        buffer[len++] = '\r';
        buffer[len++] = '\n';
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, HAL_MAX_DELAY);
}

float deg_to_radians(float degrees)
{
	return degrees * (M_PI / 180.0);
}

float gps_to_heading(float lat1, float lon1, float lat2, float lon2)
{
	// Convert latitudes and longitudes to radians
	float lat1_rad = deg_to_radians(lat1);
	float lon1_rad = deg_to_radians(lon1);
	float lat2_rad = deg_to_radians(lat2);
	float lon2_rad = deg_to_radians(lon2);

	// Calculate difference in longitude
	float delta_lon = lon2_rad - lon1_rad;

	// Calculate heading
	float y = sin(delta_lon) * cos(lat2_rad);
	float x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(delta_lon);
	float heading_rad = atan2(y, x);

	// Convert heading to degrees
	float heading_deg = heading_rad * (180.0 / M_PI);

	// Normalize heading to 0 - 360
	heading_deg = fmod(heading_deg + 360.0, 360.0);

	return heading_deg;
}

float combine_coords(int8_t degrees, int8_t minutes, int8_t seconds)
{
    float combined = 0.0;

    // Combine hours, minutes, and seconds
    combined += (float)degrees;
    combined += (float)minutes / 60.0;
    combined += (float)seconds / 3600.0;

    return combined;
}

void split_coords(float gpsData, uint8_t *degrees, uint8_t *minutes, uint8_t *seconds)
{
    // Split the degrees and minutes
    int degreesInt = (int)(gpsData / 100);
    float minutesFloat = gpsData - (degreesInt * 100);

    // Split minutes and fractional minutes (seconds)
    int minutesInt = (int)minutesFloat;
    float secondsFloat = (minutesFloat - minutesInt) * 60;

    // Round the seconds
    int secondsInt = (int)(secondsFloat + 0.5);

    // In case rounding up caused seconds to reach 60
    if(secondsInt >= 60)
    {
        secondsInt -= 60;
        minutesInt++;
    }

    // In case minutes reached 60
    if(minutesInt >= 60)
    {
        minutesInt -= 60;
        degreesInt++;
    }

    *degrees = degreesInt;
    *minutes = minutesInt;
    *seconds = secondsInt;
}


float get_distance(float lat1, float lon1, float lat2, float lon2)
{

    // Convert latitudes and longitudes to radians
    float lat1_rad = deg_to_radians(lat1);
    float lon1_rad = deg_to_radians(lon1);
    float lat2_rad = deg_to_radians(lat2);
    float lon2_rad = deg_to_radians(lon2);

    // Calculate differences in latitude and longitude
    float delta_lat = lat2_rad - lat1_rad;
    float delta_lon = lon2_rad - lon1_rad;

    // Calculate haversine of the central angle
    float a = pow(sin(delta_lat / 2.0), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(delta_lon / 2.0), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Calculate the distance using the Earth's radius
    float distance = EARTH_RADIUS_KM * c;

    return distance;
}

_Bool setBuzzer(float distance, float threshhold){

	if(distance < threshhold){
		return true;
	} else{
		return false;
	}
}

uint16_t setVibr(float headingIs, float headingMust, float rangeDeg){
	uint16_t maxTim = 999;

	float headingDiff = headingIs - headingMust;
	if(headingDiff < 0.0){
		headingDiff =+ 360.0;
	}

	if(headingDiff < rangeDeg){
		return (uint16_t)(maxTim - ((rangeDeg - headingDiff) * (maxTim / rangeDeg)));
	} else {
		return maxTim;
	}

}
