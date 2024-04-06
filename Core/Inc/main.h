/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct BME280_struct
{
	HAL_StatusTypeDef Status;
	uint8_t ID;
	double Temperature;
	double Pressure;
	double Pressure_ref;
	double Humidity;
	double Altitude;
} BME280_struct;

typedef struct MPU6050_struct
{
	HAL_StatusTypeDef Status;
	uint8_t ID;
	double Accel_X;
	double Accel_Y;
	double Accel_Z;
	double Gyro_X;
	double Gyro_Y;
	double Gyro_Z;
	double Temperature;
} MPU6050_struct;

typedef struct QMC5883_struct
{
	HAL_StatusTypeDef Status;
	uint8_t ID;
	double Xaxis;
	double Yaxis;
	double Zaxis;
	float heading;
	float compass;
	//double Temperature;
} QMC5883_struct;

typedef struct Sensors_struct
{
	struct BME280_struct BME280_External;
	struct BME280_struct BME280_Internal;
	struct MPU6050_struct MPU6050;
	struct QMC5883_struct QMC5883;
} Sensors_struct;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SD_CS_Pin GPIO_PIN_4
#define SD_CS_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_12
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_8
#define LED4_GPIO_Port GPIOA
#define GAS_VALVE_Pin GPIO_PIN_9
#define GAS_VALVE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
