/*
 * BME_lib.h
 *
 *  Created on: Jan 16, 2022
 *      Author: Aleksandra
 */

#ifndef INC_BME280_LIB_H_
#define INC_BME280_LIB_H_

#include "main.h"
#include <math.h>

#define BME280_address 0xEC  //ieksejam EC arejam EE

#define BME280_reset 0xE0
#define BME280_ID 0xD0

#define BME280_calib1 0x88
#define BME280_calib2 0xE1

#define BME280_config 0xF5
#define BME280_ctrl_meas 0xF4
#define BME280_ctrl_hum 0xF2

#define BME280_hum 0xFD
#define BME280_temp 0xFA
#define BME280_press 0xF7



typedef struct BME280_Calib_Data_struct
{
	uint16_t dig_T1; //  0x88 / 0x89
	int16_t dig_T2; //  0x8A / 0x8B
	int16_t dig_T3; //  0x8C / 0x8D
	uint16_t dig_P1; //  0x8E / 0x8F
	int16_t dig_P2; //  0x90 / 0x91
	int16_t dig_P3; //  0x92 / 0x93
	int16_t dig_P4; //  0x94 / 0x95
	int16_t dig_P5; //  0x96 / 0x97
	int16_t dig_P6; //  0x98 / 0x99
	int16_t dig_P7; //  0x9A / 0x9B
	int16_t dig_P8; //  0x9C / 0x9D
	int16_t dig_P9; //  0x9E / 0x9F
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
}BME280_Calib_Data_struct;


typedef struct BME280_pressure_data_struct
{
	uint8_t press_msb;
	uint8_t press_lsb;
	uint8_t press_xlsb;
	uint32_t ADC_P;
}BME280_pressure_data_struct;


typedef struct BME280_temperature_data_struct
{
	uint8_t temp_msb;
	uint8_t temp_lsb;
	uint8_t temp_xlsb;
	uint32_t ADC_T;
}BME280_temperature_data_struct;


typedef struct BME280_humidity_data_struct
{
	uint8_t hum_msb;
	uint8_t hum_lsb;
	uint32_t ADC_H;
}BME280_humidity_data_struct;



HAL_StatusTypeDef BME280_Init(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef BME280_Get_ID(I2C_HandleTypeDef handle, uint8_t *ptr_ID, uint16_t Timeout);
HAL_StatusTypeDef BME280_Calib_Read(I2C_HandleTypeDef handle, BME280_Calib_Data_struct *Calib_data, uint16_t Timeout);

HAL_StatusTypeDef BME280_Get_All(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef BME280_Get_Temperature(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef BME280_Get_Pressure(I2C_HandleTypeDef handle, uint16_t Timeout);
HAL_StatusTypeDef BME280_Get_Humidity(I2C_HandleTypeDef handle, uint16_t Timeout);

double BME280_T_Double(BME280_Calib_Data_struct *Calib_data);
double BME280_P_Double(BME280_Calib_Data_struct *Calib_data);
double BME280_H_Double(BME280_Calib_Data_struct *Calib_data);
double BME280_Altitude_Double(double Pressure, double Pressure_ref);

uint32_t BME280_T_Int(BME280_Calib_Data_struct *Calib_data);
uint32_t BME280_P_Int(BME280_Calib_Data_struct *Calib_data);
uint32_t BME280_H_Int(BME280_Calib_Data_struct *Calib_data);



//struct BME280_Calib_Data_struct BME280_internal_Calib_Data;
//struct BME280_Calib_Data_struct BME280_external_Calib_Data;


#endif /* INC_BME280_LIB_H_ */
