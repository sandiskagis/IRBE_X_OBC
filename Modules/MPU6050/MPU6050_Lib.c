/*
 * MPU6050_Lib.c
 *
 *  Created on: Jan 25, 2022
 *      Author: Aleksandra
 */
#include "MPU6050_Lib.h"
#include <math.h>


struct MPU6050_Read MPU6050_Accel_raw;
struct MPU6050_Read MPU6050_Gyro_raw;
int16_t Temperature_raw;



HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t data;

	// Config register complete reset (everything to 0, except sleep enable and ID)
	data = 0x80;
	status = HAL_I2C_Mem_Write(&handle, MPU6050_Address, MPU6050_PWR_MGMT_1, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;
	HAL_Delay(100);

	// Signal path and sensor reg reset
	data = 0x01;
	status = HAL_I2C_Mem_Write(&handle, MPU6050_Address, MPU6050_USER_CTRL, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;
	HAL_Delay(100);

	// Wake-up and change clock source to gyro_X reference
	data = 0x01;
	status = HAL_I2C_Mem_Write(&handle, MPU6050_Address, MPU6050_PWR_MGMT_1, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;

	// Set accelerometer range to ± 4g
	data = 0x08;
	status = HAL_I2C_Mem_Write(&handle, MPU6050_Address, MPU6050_ACCEL_CONFIG, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;

	// Set gyroscope range to ± 500°/s
	data = 0x08;
	status = HAL_I2C_Mem_Write(&handle, MPU6050_Address, MPU6050_GYRO_CONFIG, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;

	// Set sample rate to 8kHz/(1+19)=400Hz
	data = 0x13;
	status = HAL_I2C_Mem_Write(&handle, MPU6050_Address, MPU6050_SMPLRT_DIV, 1, &data, 1, Timeout);
	return status;
}



HAL_StatusTypeDef MPU6050_read_ID(I2C_HandleTypeDef handle, uint8_t *ptr_ID, uint16_t Timeout)
{
	return HAL_I2C_Mem_Read(&handle, MPU6050_Address, MPU6050_WHO_AM_I, 1, ptr_ID, 1, Timeout);
}



HAL_StatusTypeDef MPU6050_read_All(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t data[14];

	status = HAL_I2C_Mem_Read(&handle, MPU6050_Address, MPU6050_ACCEL_XOUT_H, 1, data, 14, Timeout);
	if(status != HAL_OK)
		return status;

	MPU6050_Accel_raw.xdata = data[0] << 8 | data[1];
	MPU6050_Accel_raw.ydata = data[2] << 8 | data[3];
	MPU6050_Accel_raw.zdata = data[4] << 8 | data[5];

	Temperature_raw = data[6] << 8 | data[7];

	MPU6050_Gyro_raw.xdata = data[8] << 8 | data[9];
	MPU6050_Gyro_raw.ydata = data[10] << 8 | data[11];
	MPU6050_Gyro_raw.zdata = data[12] << 8 | data[13];

	return status;
}


HAL_StatusTypeDef MPU6050_read_Accel(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t data[6];

	status = HAL_I2C_Mem_Read(&handle, MPU6050_Address, MPU6050_ACCEL_XOUT_H, 1, data, 6, Timeout);
	if(status != HAL_OK)
		return status;

	MPU6050_Accel_raw.xdata = data[0] << 8 | data[1];
	MPU6050_Accel_raw.ydata = data[2] << 8 | data[3];
	MPU6050_Accel_raw.zdata = data[4] << 8 | data[5];

	return status;
}


HAL_StatusTypeDef MPU6050_read_Temp(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t data[2];

	status = HAL_I2C_Mem_Read(&handle, MPU6050_Address, MPU6050_TEMP_OUT_H, 1, data, 2, Timeout);
	if(status != HAL_OK)
		return status;

	Temperature_raw = data[0] << 8 | data[1];
	return status;
}


HAL_StatusTypeDef MPU6050_read_Gyro(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t data[6];

	status = HAL_I2C_Mem_Read(&handle, MPU6050_Address, MPU6050_GYRO_XOUT_H, 1, data, 6, Timeout);
	if(status != HAL_OK)
		return status;

	MPU6050_Gyro_raw.xdata = data[0] << 8 | data[1];
	MPU6050_Gyro_raw.ydata = data[2] << 8 | data[3];
	MPU6050_Gyro_raw.zdata = data[4] << 8 | data[5];

	return status;
}



void MPU6050_Accel_double(MPU6050_struct *pMPU6050)
{
	pMPU6050->Accel_X = (double)MPU6050_Accel_raw.xdata / 8192.0;
	pMPU6050->Accel_Y = (double)MPU6050_Accel_raw.ydata / 8192.0;
	pMPU6050->Accel_Z = (double)MPU6050_Accel_raw.zdata / 8192.0;
}

void MPU6050_Temp_double(MPU6050_struct *pMPU6050)
{
	pMPU6050->Temperature = (double)Temperature_raw / 340 + 36.53;
}

void MPU6050_Gyro_double(MPU6050_struct *pMPU6050)
{
	pMPU6050->Gyro_X = (double)MPU6050_Gyro_raw.xdata / 65.5;
	pMPU6050->Gyro_Y = (double)MPU6050_Gyro_raw.ydata / 65.5;
	pMPU6050->Gyro_Z = (double)MPU6050_Gyro_raw.zdata / 65.5;
}
