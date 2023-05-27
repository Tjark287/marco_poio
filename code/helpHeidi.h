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

#define EARTH_RADIUS_KM 6371.0

void serialPrint(const char *str, ...);
float deg_to_radians(float degrees);
float gps_to_heading(float lat1, float lon1, float lat2, float lon2);
float combine_coords(int8_t degrees, int8_t minutes, int8_t seconds);
void split_coords(float gpsData, uint8_t *degrees, uint8_t *minutes, uint8_t *seconds);
float get_distance(float lat1, float lon1, float lat2, float lon2);
_Bool setBuzzer(float distance, float threshhold);
uint16_t setVibr(float headingIs, float headingMust, float rangeDeg);

#endif /* INC_HELPHEIDI_H_ */
