/**
  ******************************************************************************
  * @file           : helpHeidi.h
  * @brief          : Helper functions for MARCO POIo device
  ******************************************************************************
  * @attention
  * helping HEIDIs since 2023
  * Author: MTFinding
  ******************************************************************************
*/

#ifndef INC_HELPHEIDI_H_
#define INC_HELPHEIDI_H_

// UART Handle (Serial Port COM)
extern UART_HandleTypeDef huart2;

#define EARTH_RADIUS_M 6371000.0

void serialPrint(const char *str, ...);
float to_radians(float degrees);
float to_degrees(float radians);
float gps_to_heading(float latitude1, float longitude1, float latitude2, float longitude2);
float combine_coords(uint8_t degrees, uint8_t minutes, uint8_t seconds, uint8_t hundredths);
void split_coords(float gpsData, uint8_t *degrees, uint8_t *minutes, uint8_t *seconds, uint8_t *hundredths);
float get_distance(float lat1, float lon1, float lat2, float lon2);
_Bool setBuzzer(float distance, float threshhold);
uint16_t setVibr(float headingIs, float headingMust, float rangeDeg);
void clearBuffer(uint8_t* buffer, size_t bufferSize);

#endif /* INC_HELPHEIDI_H_ */
