/**
  ******************************************************************************
  * @file           : gps.h
  * @brief          : mini library for L76-M33 GPS receiver
  ******************************************************************************
  * @attention
  * Author: MTFinding
  ******************************************************************************
*/

#ifndef INC_GPS_H_
#define INC_GPS_H_

extern UART_HandleTypeDef huart1;

#define GPS_receiveLen 500

_Bool get_gnrmc_line(char *input, char *output, int outputSize);
_Bool GPS_process(char *gpsData, float *latitude, float *longitude, uint8_t *numSatellites);


#endif /* INC_GPS_H_ */
