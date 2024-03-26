/*
 * BME280_lib.c
 *
 *  Created on: Jan 16, 2022
 *      Author: Aleksandra
 */
#include "BME280_lib.h"
#include "main.h"
#include <math.h>

int32_t t_fine;

struct BME280_pressure_data_struct Pressure_Data;
struct BME280_temperature_data_struct Temperature_Data;
struct BME280_humidity_data_struct Humidity_Data;


HAL_StatusTypeDef BME280_Init(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t data;

	// RESET
	data = 0xB6;
	status = HAL_I2C_Mem_Write(&handle, BME280_address, BME280_reset, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;

	// Config data
	data = 0b10000000;
	status = HAL_I2C_Mem_Write(&handle, BME280_address, BME280_config, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;

	// Humidity measure settings
	data = 0b00000011;
	status = HAL_I2C_Mem_Write(&handle, BME280_address, BME280_ctrl_hum, 1, &data, 1, Timeout);
	if(status != HAL_OK)
		return status;

	// Temperature/Pressure measure settings
	data = 0b10001111;
	status = HAL_I2C_Mem_Write(&handle, BME280_address, BME280_ctrl_meas, 1, &data, 1, Timeout);
	return status;
}


HAL_StatusTypeDef BME280_Get_ID(I2C_HandleTypeDef handle, uint8_t *ptr_ID, uint16_t Timeout)
{
	return HAL_I2C_Mem_Read(&handle, BME280_address, BME280_ID, 1, ptr_ID, 1, Timeout);
}


HAL_StatusTypeDef BME280_Calib_Read(I2C_HandleTypeDef handle, BME280_Calib_Data_struct *Calib_data, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t calib0[26];
	uint8_t calib1[16];

	status = HAL_I2C_Mem_Read(&handle, BME280_address, BME280_calib1, 1, calib0, 26, Timeout);
	status = HAL_I2C_Mem_Read(&handle, BME280_address, BME280_calib2, 1, calib1, 16, Timeout);
	if(status != HAL_OK)
		return status;

	Calib_data->dig_T1 = calib0[0];
	Calib_data->dig_T1 += calib0[1] << 8;

	Calib_data->dig_T2 = calib0[2];
	Calib_data->dig_T2 += calib0[3] << 8;

	Calib_data->dig_T3 = calib0[4];
	Calib_data->dig_T3 += calib0[5] << 8;

	Calib_data->dig_P1 = calib0[6];
	Calib_data->dig_P1 += calib0[7] << 8;

	Calib_data->dig_P2 = calib0[8];
	Calib_data->dig_P2 += calib0[9] << 8;

	Calib_data->dig_P3 = calib0[10];
	Calib_data->dig_P3 += calib0[11] << 8;

	Calib_data->dig_P4 = calib0[12];
	Calib_data->dig_P4 += calib0[13] << 8;

	Calib_data->dig_P5 = calib0[14];
	Calib_data->dig_P5 += calib0[15] << 8;

	Calib_data->dig_P6 = calib0[16];
	Calib_data->dig_P6 += calib0[17] << 8;

	Calib_data->dig_P7 = calib0[18];
	Calib_data->dig_P7 += calib0[19] << 8;

	Calib_data->dig_P8 = calib0[20];
	Calib_data->dig_P8 += calib0[21] << 8;

	Calib_data->dig_P9 = calib0[22];
	Calib_data->dig_P9 += calib0[23] << 8;

	Calib_data->dig_H1 = calib0[25];

	Calib_data->dig_H2 = calib1[0];
	Calib_data->dig_H2 += calib1[1] << 8;

	Calib_data->dig_H3 = calib1[2];

	Calib_data->dig_H4 = calib1[3] << 4;
	Calib_data->dig_H4 += calib1[4] & 0b00001111;

	Calib_data->dig_H5 = calib1[4] >> 4;
	Calib_data->dig_H5 += calib1[5] << 4;

	Calib_data->dig_H6 = calib1[6];

	return status;
}


HAL_StatusTypeDef BME280_Get_All(I2C_HandleTypeDef handle, uint16_t Timeout){
	HAL_StatusTypeDef status;
	uint8_t measure[8];

	status = HAL_I2C_Mem_Read(&handle, BME280_address, BME280_press, 1, measure, 8, Timeout);
	if(status != HAL_OK)
		return status;

	Pressure_Data.press_msb = measure[0];
	Pressure_Data.press_lsb = measure[1];
	Pressure_Data.press_xlsb = measure[2];

	Temperature_Data.temp_msb = measure[3];
	Temperature_Data.temp_lsb = measure[4];
	Temperature_Data.temp_xlsb = measure[5];

	Humidity_Data.hum_msb = measure[6];
	Humidity_Data.hum_lsb = measure[7];

	Temperature_Data.ADC_T = ((uint32_t)Temperature_Data.temp_msb << 12) | ((uint32_t)Temperature_Data.temp_lsb << 4) | ((uint32_t)Temperature_Data.temp_xlsb >> 4);
	Pressure_Data.ADC_P = ((uint32_t)Pressure_Data.press_msb << 12) | ((uint32_t)Pressure_Data.press_lsb << 4) | ((uint32_t)Pressure_Data.press_xlsb >> 4);
	Humidity_Data.ADC_H = ((uint32_t)Humidity_Data.hum_msb << 8) | ((uint32_t)Humidity_Data.hum_lsb);
	return status;
}


HAL_StatusTypeDef BME280_Get_Temperature(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t measure[3];

	status = HAL_I2C_Mem_Read(&handle, BME280_address, BME280_temp, 1, measure, 3, Timeout);
	if(status != HAL_OK)
		return status;

	Temperature_Data.temp_msb = measure[0];
	Temperature_Data.temp_lsb = measure[1];
	Temperature_Data.temp_xlsb = measure[2];

	Temperature_Data.ADC_T = ((uint32_t)Temperature_Data.temp_msb << 12) | ((uint32_t)Temperature_Data.temp_lsb << 4) | ((uint32_t)Temperature_Data.temp_xlsb >> 4);
	return status;
}


HAL_StatusTypeDef BME280_Get_Pressure(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t measure[3];

	status = HAL_I2C_Mem_Read(&handle, BME280_address, BME280_press, 1, measure, 3, Timeout);
	if(status != HAL_OK)
		return status;

	Pressure_Data.press_msb = measure[0];
	Pressure_Data.press_lsb = measure[1];
	Pressure_Data.press_xlsb = measure[2];

	Pressure_Data.ADC_P = ((uint32_t)Pressure_Data.press_msb << 12) | ((uint32_t)Pressure_Data.press_lsb << 4) | ((uint32_t)Pressure_Data.press_xlsb >> 4);
	return status;
}


HAL_StatusTypeDef BME280_Get_Humidity(I2C_HandleTypeDef handle, uint16_t Timeout)
{
	HAL_StatusTypeDef status;
	uint8_t measure[3];

	status = HAL_I2C_Mem_Read(&handle, BME280_address, BME280_hum, 1, measure, 2, Timeout);
	if(status != HAL_OK)
		return status;

	Humidity_Data.hum_msb = measure[0];
	Humidity_Data.hum_lsb = measure[1];

	Humidity_Data.ADC_H = ((uint32_t)Humidity_Data.hum_msb << 8) | ((uint32_t)Humidity_Data.hum_lsb);
	return status;
}


double BME280_T_Double (BME280_Calib_Data_struct *Calib_data)
{
//	BME280_Temperature(handle);
	double var1, var2, t;

	var1 = (((double) Temperature_Data.ADC_T)/16384.0 - ((double) Calib_data->dig_T1)/1024.0) * ((double) Calib_data->dig_T2);
	var2 = ((((double) Temperature_Data.ADC_T)/131072.0 - ((double) Calib_data->dig_T1)/8192.0) * (((double) Temperature_Data.ADC_T)/131072.0 - ((double) Calib_data->dig_T1)/8192.0)) * ((double) Calib_data->dig_T3);

	t_fine =  (var1 + var2);
	t = (var1 + var2)/5120.0;

	return t;
}

double BME280_P_Double(BME280_Calib_Data_struct *Calib_data)
{
//	BME280_Pressure(handle);
	double var1, var2, p;

	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double) Calib_data->dig_P6) / 32768.0;
	var2 = var2 + var1 + ((double) Calib_data->dig_P5) * 2.0;
	var2 = (var2/4.0) + (((double) Calib_data->dig_P4) * 65536.0);
	var1 = (((double) Calib_data->dig_P3) * var1 * var1 / 524288.0 + ((double) Calib_data->dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double) Calib_data->dig_P1);

	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576.0 - (double) Pressure_Data.ADC_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;

	var1 = ((double) Calib_data->dig_P9) * p* p / 2147483648.0;
	var2 = p * ((double) Calib_data->dig_P8) / 32768.0;

	p = p + (var1 + var2 + ((double) Calib_data->dig_P7)) / 16.0;
	p = (p/1000);

	return p;
}

double BME280_H_Double(BME280_Calib_Data_struct *Calib_data)
{
//	BME280_Humidity(handle);
	long double var_H;

	var_H = (((double) t_fine) - 76800.0);
	var_H = (((double) Humidity_Data.ADC_H) - (((double)Calib_data->dig_H4)*64.0 + ((double)Calib_data->dig_H5)/16384.0*var_H)) *
			(((double) Calib_data->dig_H2)/65536.0 * (1.0 + ((double) Calib_data->dig_H6) / 67108864.0*var_H * (1.0 + ((double) Calib_data->dig_H3)/67108864.0*var_H)));
	var_H *= (1.0 - ((double)Calib_data->dig_H1)*var_H/524288.0);

	if (var_H > 100.0){
		var_H = 100.0;
	}
	else if (var_H < 0.0){
		var_H = 0.0;
	}

	return var_H;
}


double BME280_Altitude_Double(double Pressure, double Pressure_ref)
{
	long double var1;
	var1 = pow(Pressure/Pressure_ref, 1/5.255);
	return 44330*(1-var1);
}


uint32_t BME280_T_Int(BME280_Calib_Data_struct *Calib_data){
//	BME280_Temperature(handle);
	uint32_t var1, var2, T;

	var1 = ((((Temperature_Data.ADC_T >> 3) - ((uint32_t) Calib_data->dig_T1 << 1))) * ((uint32_t) Calib_data->dig_T2)) >> 11;
	var2 = (((((Temperature_Data.ADC_T >> 4) - ((uint32_t) Calib_data->dig_T1)) * ((Temperature_Data.ADC_T >> 4) - ((uint32_t) Calib_data->dig_T1))) >> 12) *
	((uint32_t) Calib_data->dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;

	return T;
}
uint32_t BME280_P_Int(BME280_Calib_Data_struct *Calib_data){
//	BME280_Pressure(handle);
	uint64_t var1, var2;
	uint64_t p;

	var1 = ((uint64_t)t_fine) - 128000;
	var2 = var1 * var1 * ((uint64_t) Calib_data->dig_P6);
	var2 = var2 + ((var1* (uint64_t) Calib_data->dig_P5) << 17);
	var2 = var2 + (((uint64_t) Calib_data->dig_P4) << 35);
	var1 = ((var1 * var1 * (uint64_t)Calib_data->dig_P3) >> 8) + ((var1 * (uint64_t)Calib_data->dig_P2) << 12);
	var1 = (((((uint64_t)1) << 47 ) + var1)) * ((uint64_t) Calib_data->dig_P1) >> 33;

	if (var1 == 0){
		return 0; // avoid exception caused by division by zero
	}

	p = 1048576 - Pressure_Data.ADC_P;
	p = (((p << 31) - var2 ) * 3125) / var1;

	var1 = (((uint64_t) Calib_data->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 =  ((uint64_t) Calib_data->dig_P8 * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((uint64_t) Calib_data->dig_P7) << 4);

	return (uint32_t)p;

}

uint32_t BME280_H_Int(BME280_Calib_Data_struct *Calib_data){
//	BME280_Humidity(handle);
	int32_t var_H;
	var_H = (t_fine - ((int32_t) 76800));
	var_H = (((((Humidity_Data.ADC_H << 14) - (((int32_t) Calib_data->dig_H4) << 20) - (((int32_t) Calib_data->dig_H5) * var_H)) + ((int32_t)16384)) >> 15) * (((((((var_H * ((int32_t) Calib_data->dig_H6)) >> 10) * (((var_H + ((int32_t) Calib_data->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)Calib_data->dig_H2) + 8192) >> 14));
	var_H = (var_H - (((((var_H >> 15) * (var_H >> 15)) >> 7) * ((int32_t)Calib_data->dig_H1)) >> 4));

	return (uint32_t)(var_H >> 12);
}
