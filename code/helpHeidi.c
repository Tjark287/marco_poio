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
    uint8_t len = vsnprintf(buffer, sizeof(buffer), str, args);
    va_end(args);

    // Add the newline character
    if (len > 0 && len < sizeof(buffer) - 1) {
        buffer[len++] = '\r';
        buffer[len++] = '\n';
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 10);
}

float to_radians(float degrees) {
    return degrees * M_PI / 180.0;
}

float to_degrees(float radians) {
    return radians * 180.0 / M_PI;
}

float gps_to_heading(float latitude1, float longitude1, float latitude2, float longitude2) {
    // Convert latitude and longitude values to radians
    latitude1 = to_radians(latitude1);
    longitude1 = to_radians(longitude1);
    latitude2 = to_radians(latitude2);
    longitude2 = to_radians(longitude2);

    // Calculate differences
    float dLongitude = longitude2 - longitude1;

    // Calculate X and Y
    float X = cos(latitude2) * sin(dLongitude);
    float Y = cos(latitude1) * sin(latitude2) - sin(latitude1) * cos(latitude2) * cos(dLongitude);

    // Calculate bearing
    float bearing = atan2(X, Y);

    // Convert to degrees
    bearing = to_degrees(bearing);

    // Normalize bearing to a value between 0 degree and 360 degrees
    if(bearing < 0)
        bearing += 360;

    return bearing;
}

float combine_coords(uint8_t degrees, uint8_t minutes, uint8_t seconds, uint8_t hundredths)
{
    float combined = 0.0;

    // Combine degrees, minutes, seconds and hundredths
    combined += (float)degrees;
    combined += (float)minutes / 60.0;
    combined += ((float)seconds + (float)hundredths / 100.0) / 3600.0;

    return combined;
}


void split_coords(float gpsData, uint8_t *degrees, uint8_t *minutes, uint8_t *seconds, uint8_t *hundredths)
{
	uint8_t degreesInt, minutesInt, secondsInt, hundredthsInt;

	degreesInt = (uint8_t) gpsData;
	float minutes_temp = (gpsData - degreesInt) * 60;
	minutesInt = (uint8_t) minutes_temp;
	float seconds_temp = (minutes_temp - minutesInt) * 60;
	secondsInt = (uint8_t) seconds_temp;
	hundredthsInt = (uint8_t) ((seconds_temp - secondsInt) * 100);

    *degrees = degreesInt;
    *minutes = minutesInt;
    *seconds = secondsInt;
    *hundredths = hundredthsInt;
}



float get_distance(float lat1, float lon1, float lat2, float lon2)
{

    // Convert latitudes and longitudes to radians
    float lat1_rad = to_radians(lat1);
    float lon1_rad = to_radians(lon1);
    float lat2_rad = to_radians(lat2);
    float lon2_rad = to_radians(lon2);

    // Calculate differences in latitude and longitude
    float delta_lat = lat2_rad - lat1_rad;
    float delta_lon = lon2_rad - lon1_rad;

    // Calculate haversine of the central angle
    float a = pow(sin(delta_lat / 2.0), 2) + cos(lat1_rad) * cos(lat2_rad) * pow(sin(delta_lon / 2.0), 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    // Calculate the distance using the Earth's radius
    float distance = EARTH_RADIUS_M * c;

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

    float headingDiff = fmodf(fabsf(headingIs - headingMust), 360.0);
    if (headingDiff > 180.0) {
        headingDiff = 360.0 - headingDiff;
    }

    if(headingDiff <= rangeDeg){
        return (uint16_t)(maxTim - ((headingDiff / rangeDeg) * maxTim));
    } else {
        return 0;
    }
}

void clearBuffer(uint8_t* buffer, size_t bufferSize)
{
    memset(buffer, 0, bufferSize);
}
