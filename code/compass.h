/**
  ******************************************************************************
  * @file           : compass.h
  * @brief          : mini library for MMC3416xPJ magnetometer
  ******************************************************************************
  * @attention
  * Author: MTFinding
  ******************************************************************************
*/

#ifndef INC_COMPASS_H_
#define INC_COMPASS_H_

extern I2C_HandleTypeDef hi2c1;

#define CM_I2Caddr 		0b01100000
#define CM_FREQ_1_5		0x02
#define CM_FREQ_13		0x06
#define CM_FREQ_25		0x0A
#define CM_FREQ_50		0x0E

#define CM_xAxisLow		0x00
#define CM_yAxisLow		0x02
#define CM_zAxisLow		0x04

_Bool CM_init();
_Bool CM_writeReg(uint8_t adr, uint8_t val);
_Bool CM_readMag(uint16_t* X, uint16_t* Y, uint16_t* Z);
_Bool CM_measure();
void CM_waitDataRdy();
float CM_getheading();


#endif /* INC_COMPASS_H_ */
