/**
  ******************************************************************************
  * @file           : compass.c
  * @brief          : mini library for MMC3416xPJ magnetometer
  ******************************************************************************
  * @attention
  * Author: MTFinding
  ******************************************************************************
*/

#include <main.h>
#include <compass.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

static float offset[3]; // global offset variable

_Bool CM_writeReg(uint8_t adr, uint8_t val)
{
	uint8_t sendData[2] = {adr, val};
	if (HAL_I2C_Master_Transmit(&hi2c1, CM_I2Caddr, sendData, 2, HAL_MAX_DELAY) == HAL_OK){
		return true;
	}

	return false;
}

uint8_t CM_readReg(uint8_t adr)
{
	uint8_t recData;
	HAL_I2C_Master_Transmit(&hi2c1, CM_I2Caddr, &adr, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, CM_I2Caddr, &recData, 1, HAL_MAX_DELAY);
	return recData;
}

_Bool CM_init()
{
	uint16_t data0_i[3], data1_i[3];
	float data0_f[3], data1_f[3];

	if (!CM_writeReg(0x07, 0x80)) return false;	// refill CAP
	HAL_Delay(60);
	if (!CM_writeReg(0x07, 0x20)) return false;	// SET
	HAL_Delay(10);

	// read data for offset calc
	if(!CM_readMag(&data0_i[0], &data0_i[1], &data0_i[2])) return false;
	for(uint8_t i = 0; i<3;i++){
		data0_f[i] = 0.48828125 * (float)data0_i[i];
	}

	if (!CM_writeReg(0x07, 0x80)) return false;	// refill CAP
	HAL_Delay(60);
	if (!CM_writeReg(0x07, 0x40)) return false;	// RESET
	HAL_Delay(10);

	// read data for offset calc
	if(!CM_readMag(&data1_i[0], &data1_i[1], &data1_i[2])) return false;
	for(uint8_t i = 0; i<3;i++){
		data1_f[i] = 0.48828125 * (float)data1_i[i];
	}
	for(uint8_t i = 0; i<3;i++){
		offset[i] = (data0_f[i] + data1_f[i]) * 0.5;
	}

	if (!CM_writeReg(0x07, 0x40)) return false;	// RESET
	HAL_Delay(10);
	if (!CM_writeReg(0x08, 0x00)) return false;	// write config
	HAL_Delay(10);
	return true;
}

_Bool CM_measure()
{
    if (!CM_writeReg(0x07, 0x01)) return false;
    return true;
}

void CM_waitDataRdy()
{
	while(!(CM_readReg(0x06) & 0x01)){
	}
}

_Bool CM_readMag(uint16_t* X, uint16_t* Y, uint16_t* Z)
{
	HAL_StatusTypeDef ack = 0;
	uint8_t start = 0x00;
    uint8_t rxData[6];

	// initiate measurement
    if (!CM_writeReg(0x07, 0x01)) return false;

    // wait until data ready
	while(!(CM_readReg(0x06) & 0x01)){
	}
	// read data
	ack += HAL_I2C_Master_Transmit(&hi2c1, CM_I2Caddr, &start, 1, HAL_MAX_DELAY);
	ack += HAL_I2C_Master_Receive(&hi2c1, CM_I2Caddr, rxData, 6, HAL_MAX_DELAY);
	if (ack != 0) return false;

	*X = (rxData[1] << 8) | rxData[0];
	*Y = (rxData[3] << 8) | rxData[2];
	*Z = (rxData[5] << 8) | rxData[4];

	return true;
}

float CM_getheading()
{
    uint16_t Xraw, Yraw, Zraw;

    // fetch data
    if(!CM_readMag(&Xraw, &Yraw, &Zraw)){
    	return -1.0f;
    }

    // convert to float and add offset
    float X = 0.48828125 * (float)Xraw - offset[0];
    float Y = 0.48828125 * (float)Yraw - offset[1];
//  float Z = 0.48828125 * (float)Zraw - offset[2];

    // div 0
    if(X == 0 || Y == 0){
    	return -1.0f;
    }

    // calc heading
    float heading_rad = atan2(Y, X);

    // rad to degrees
    float heading_deg = heading_rad * 180 / M_PI;

    // add declination
    heading_deg += DECLINATION_VIENNA;

    // normalize heading
    if (heading_deg < 0) {
        heading_deg += 360;
    }

    return heading_deg;
}
