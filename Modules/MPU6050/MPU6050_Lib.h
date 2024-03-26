/*
 * MPU6050_Lib.h
 *
 *  Created on: Jan 25, 2022
 *      Author: Aleksandra
 */

#ifndef INC_MPU6050_LIB_H_
#define INC_MPU6050_LIB_H_

#include "main.h"
#include <math.h>

#define MPU6050_Address 			0xD0

#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_WHO_AM_I		    0x75

typedef struct MPU6050_Read
{
	int16_t xdata;
	int16_t ydata;
	int16_t zdata;
}MPU6050_Read;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef MPU6050_read_ID(I2C_HandleTypeDef handle, uint8_t *ptr_ID, uint16_t Timeout);
HAL_StatusTypeDef MPU6050_read_All(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef MPU6050_read_Accel(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef MPU6050_read_Temp(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef MPU6050_read_Gyro(I2C_HandleTypeDef handle, uint16_t Timeout);

void MPU6050_Accel_double(MPU6050_struct *pMPU6050);
void MPU6050_Temp_double(MPU6050_struct *pMPU6050);
void MPU6050_Gyro_double(MPU6050_struct *pMPU6050);

#endif /* INC_MPU6050_LIB_H_ */
