/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BME280_lib.h"
#include "MPU6050_Lib.h"
#include "QMC5883_lib.h" //1.library
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_short_timeout 20
#define I2C_long_timeout 200
#define PRESSURE_REFERENCE 100	//kpa at sea level, varies by location and weather
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
Sensors_struct Sensors;
uint8_t USART1_RxBuffer [6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void Get_BME280_in_all_readings();
//void Get_BME280_ex_all_readings();
void Get_MPU6050_all_readings();
void Get_QMC5883_all_readings();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS	FS;							// fil system object structure (FATFS)
FATFS	*pFS;						// Pointer to fil system object structure
FIL		fil;						// fil object structure (FIL)

char buffer [100];
char Data_to_send[50];
uint8_t Rx_command1[2] = {0};
uint8_t Rx_command3[2] = {0};
char tx_data[] = "hello";

float heading;


char rx_buffer[1]; // Buffer to store received characters

typedef struct
{
	volatile FRESULT mount;				// To store return status code
	volatile FRESULT open;				// If SD card mounted save 1, if not save 0
	volatile FRESULT freeSpace;
	volatile FRESULT statusSync;

	volatile uint8_t statuss;

	DWORD mytotalSpace;
	DWORD myfreeSpace;

	DWORD	freeCluster;

}MYfilResult;

MYfilResult Data;

uint8_t	MY_SDCard_Mount ();
uint8_t	MY_SDCard_Open();
uint8_t MY_SDCard_FreeSpace();
MYfilResult MY_SDCard_SetUp ();

RTC_DateTypeDef	systemRTCDate;
RTC_TimeTypeDef	systemRTCTime;

struct
{
	uint8_t state;
	uint8_t nextState;
}SM;

enum
{
	SM_STATE_READ_SENSORS,
	SM_STATE_WRITE_TO_SD_CARD,
	SM_STATE_SEND_DATA_TO_COM
};

struct BME280_Calib_Data_struct BME280_internal_Calib_Data;
//struct BME280_Calib_Data_struct BME280_external_Calib_Data;

uint8_t tim3Flag = 0;
uint16_t myCnt[10] = {0};

void Get_BME280_in_all_readings()
{
	Sensors.BME280_Internal.Status = BME280_Get_ID(hi2c1, &Sensors.BME280_Internal.ID, I2C_short_timeout);
	if(Sensors.BME280_Internal.Status != HAL_OK)
		return;
	Sensors.BME280_Internal.Status = BME280_Get_All(hi2c1, I2C_long_timeout);
	if(Sensors.BME280_Internal.Status != HAL_OK)
		return;

	Sensors.BME280_Internal.Temperature = BME280_T_Double(&BME280_internal_Calib_Data);
	Sensors.BME280_Internal.Pressure = BME280_P_Double(&BME280_internal_Calib_Data);
	Sensors.BME280_Internal.Humidity = BME280_H_Double(&BME280_internal_Calib_Data);
	Sensors.BME280_Internal.Altitude = BME280_Altitude_Double(Sensors.BME280_Internal.Pressure, Sensors.BME280_Internal.Pressure_ref);
}

/*
void Get_BME280_ex_all_readings()
{
	Sensors.BME280_External.Status = BME280_Get_ID(hi2c2, &Sensors.BME280_External.ID, I2C_short_timeout);
	if(Sensors.BME280_External.Status != HAL_OK)
		return;
	Sensors.BME280_External.Status = BME280_Get_All(hi2c2, I2C_long_timeout);
	if(Sensors.BME280_External.Status != HAL_OK)
		return;

	Sensors.BME280_External.Temperature = BME280_T_Double(&BME280_external_Calib_Data);
	Sensors.BME280_External.Pressure = BME280_P_Double(&BME280_external_Calib_Data);
	Sensors.BME280_External.Humidity = BME280_H_Double(&BME280_external_Calib_Data);
	Sensors.BME280_External.Altitude = BME280_Altitude_Double(Sensors.BME280_External.Pressure, Sensors.BME280_External.Pressure_ref);
}
*/
void Get_MPU6050_all_readings()
{
	Sensors.MPU6050.Status = MPU6050_read_ID(hi2c1, &Sensors.MPU6050.ID, I2C_short_timeout);
	myCnt[2]++;
	if(Sensors.MPU6050.Status != HAL_OK)
		return;
	Sensors.MPU6050.Status = MPU6050_read_All(hi2c1, I2C_long_timeout);
	myCnt[3]++;
	if(Sensors.MPU6050.Status != HAL_OK)
		return;

	MPU6050_Accel_double(&Sensors.MPU6050);
	myCnt[4]++;
	MPU6050_Gyro_double(&Sensors.MPU6050);
	MPU6050_Temp_double(&Sensors.MPU6050);
}

QMC_t QMC_Data; //1.library and 2.library
float Compas_Value; //2.library






void Get_QMC5883_all_readings()
{

	if(QMC_read(&QMC_Data)==0)
	{
		//HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
		Compas_Value=QMC_Data.heading;
	}
	else
	{
		//HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
	}
	//HAL_Delay(50);
	HAL_Delay(I2C_long_timeout);

}


// Function to clear the buffer
void ClearBuffer(uint8_t *buffer, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        buffer[i] = '\0';
    }
}

// Make sure to call this function to start the first reception
void StartReception()
{
    ClearBuffer(USART1_RxBuffer, 6);
    HAL_UART_Receive_IT(&huart1, USART1_RxBuffer, 5);
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  /*------------------Sensors init-----------------------*/
//  	Sensors.BME280_Internal.Pressure_ref = PRESSURE_REFERENCE;
//	Sensors.BME280_External.Pressure_ref = PRESSURE_REFERENCE;

  	/* Wait for Sensor power-on */
  	HAL_Delay(1000);

  	Sensors.BME280_Internal.Status = BME280_Get_ID(hi2c1, &Sensors.BME280_Internal.ID, I2C_short_timeout);
  	if (Sensors.BME280_Internal.Status == HAL_OK)
  	{
  		Sensors.BME280_Internal.Status = BME280_Init(hi2c1, I2C_short_timeout);
  		Sensors.BME280_Internal.Status = BME280_Calib_Read(hi2c1, &BME280_internal_Calib_Data, I2C_long_timeout);
  		Get_BME280_in_all_readings();
  	}

/*
  	Sensors.BME280_External.Status = BME280_Get_ID(hi2c2, &Sensors.BME280_External.ID, I2C_short_timeout);
  	if (Sensors.BME280_External.Status == HAL_OK)
  	{
  		Sensors.BME280_External.Status = BME280_Init(hi2c2, I2C_short_timeout);
  		Sensors.BME280_External.Status = BME280_Calib_Read(hi2c2, &BME280_external_Calib_Data, I2C_long_timeout);
  		Get_BME280_ex_all_readings();
  	}
*/
  	Sensors.MPU6050.Status = MPU6050_read_ID(hi2c1, &Sensors.MPU6050.ID, I2C_short_timeout);
  	if (Sensors.MPU6050.Status == HAL_OK)
  	{
  		Sensors.MPU6050.Status = MPU6050_Init(hi2c1, I2C_short_timeout);
  		Get_MPU6050_all_readings();
  	}




/*
  	//if (Sensors.QMC5883.Status == HAL_OK)
  	//{
  	//Sensors.QMC5883.Status = QMC_init(&QMC_Data, &hi2c2, I2C_short_timeout);
  	//QMC_init(&QMC_Data, &hi2c2, 100);
  	//QMC_readHeading(&QMC_Data);
  	//QMC_read(&QMC_Data);
  	//Get_QMC5883_all_readings();
  	//}
*/ //1.library




  	Sensors.QMC5883.Status = QMC_init(&QMC_Data, &hi2c1, 200);



  	Data = MY_SDCard_SetUp();
  	HAL_TIM_Base_Start_IT(&htim3);
  	//HAL_UART_Receive_IT(&huart1, USART1_RxBuffer, 10);
  	//HAL_UART_Receive_IT(&huart3, Rx_command3, 1);
  	StartReception();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



	switch(SM.state){
		case SM_STATE_READ_SENSORS:
			if(tim3Flag == 1){
				Get_BME280_in_all_readings();
				//Get_BME280_ex_all_readings();
	  	  		Get_MPU6050_all_readings();
	  	  		Get_QMC5883_all_readings();
	  	  		//QMC_read(&QMC_Data);

	  	  		//HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data), HAL_MAX_DELAY);

	  	  		HAL_RTC_GetTime(&hrtc, &systemRTCTime, RTC_FORMAT_BIN);
	  	  		HAL_RTC_GetDate(&hrtc, &systemRTCDate, RTC_FORMAT_BIN);
	  	  		tim3Flag = 0;
	  	  		SM.nextState= SM_STATE_WRITE_TO_SD_CARD;
	  	  		SM.state = SM.nextState;
	  	  		break;
	  	  	}
	  	  	//SM.nextState= SM_STATE_READ_SENSORS;
	  	  	SM.nextState= SM_STATE_WRITE_TO_SD_CARD;
	  	  	SM.state = SM.nextState;
	  	 break;
	  	 case SM_STATE_WRITE_TO_SD_CARD:
	  		 if((Data.statuss == 1) && (Data.statusSync == FR_OK)){
	  			 char TempStr[100];
	  	  		 //sprintf(TempStr, "%9.2f  %9.2f  %9.2f  %9.2f  \n", Sensors.BME280_Internal.Temperature, Sensors.BME280_Internal.Humidity, Sensors.MPU650.Gyro_X, Sensors.MPU650.Gyro_Y);
	  			 sprintf(TempStr, "%d\t%d\t%d\t%3.2f\t%3.2f\n",systemRTCTime.Hours, systemRTCTime.Minutes, systemRTCTime.Seconds, Sensors.BME280_Internal.Temperature, Sensors.BME280_Internal.Pressure);
	  			 f_printf(&fil, TempStr);
	  	  		 Data.statusSync = f_sync(&fil);
	  	  		 myCnt[1]++;
	  	  		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	  	  		 //SM.nextState= SM_STATE_SEND_DATA_TO_COM;
	  	  		 SM.nextState= SM_STATE_READ_SENSORS;
	  	  		 SM.state = SM.nextState;
	  	  		 break;
	  	  	  }
	  	  	  //SM.nextState= SM_STATE_SEND_DATA_TO_COM;
	  		  SM.nextState= SM_STATE_READ_SENSORS;
	  	  	  SM.state = SM.nextState;
	  	 break;
	  	 case SM_STATE_SEND_DATA_TO_COM:
	  		 memset(Data_to_send, 0, sizeof(Data_to_send));
	  		 char TempStr[20];
	  		 sprintf(TempStr, "%.2f", Sensors.BME280_Internal.Temperature);
	  		 strcat(Data_to_send, TempStr);
	  		 HAL_UART_Transmit_IT(&huart1,Data_to_send, 5);
	  		 SM.nextState= SM_STATE_READ_SENSORS;
	  		 SM.state = SM.nextState;
	  		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  	 default:
	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);

	}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x003038FF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
  sDate.Month = RTC_MONTH_FEBRUARY;
  sDate.Date = 0x25;
  sDate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_CS_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|LED0_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin|GAS_VALVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SD_CS_Pin LED4_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 LED0_Pin LED1_Pin LED2_Pin
                           LED3_Pin GAS_VALVE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LED0_Pin|LED1_Pin|LED2_Pin
                          |LED3_Pin|GAS_VALVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

MYfilResult MY_SDCard_SetUp()
{
	MYfilResult SD_FR;

	SD_FR.mount = f_mount(&FS, "", 0);
	SD_FR.open = f_open(&fil, "IRBEX.txt", FA_OPEN_ALWAYS | FA_WRITE);
	if((SD_FR.mount == FR_OK) && (SD_FR.open == FR_OK))
	{
		SD_FR.freeSpace = f_getfree("", &SD_FR.freeCluster, &pFS);
		if(SD_FR.freeSpace == FR_OK)
		{
			SD_FR.mytotalSpace = (pFS -> n_fatent - 2) * (pFS -> csize);
			SD_FR.myfreeSpace = SD_FR.freeCluster * (pFS -> csize);

			SD_FR.statuss = 1;
		}

		if(SD_FR.statuss == 1)
		{
			//Write a meta date of data logger text fil
			f_printf(&fil, "IRBEX DATA LOGGER.\n");
			HAL_RTC_GetDate(&hrtc, &systemRTCDate, RTC_FORMAT_BIN);
			f_printf(&fil, "Date: %d.%d.%d\n", systemRTCDate.Date, systemRTCDate.Month, systemRTCDate.Year);
			f_printf(&fil, "SD Card memory total space = %d & free space = %d\n", SD_FR.mytotalSpace, SD_FR.myfreeSpace);
			f_printf(&fil, "Hours\tMinutes\tSeconds\tInternal_Temp\tInternal_Pressure\t");
			SD_FR.statusSync = f_sync(&fil);
			SD_FR.statusSync = f_sync(&fil);
		}
	}
	return SD_FR;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		tim3Flag = 1;
		myCnt[0]++;
	}
}





// Function to convert double to string
char* doubleToString(double value) {
    static char buffer[20];
    snprintf(buffer, 20, "%.2f", value); // Adjust precision as needed
    return buffer;
}







void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{




//	if (huart == &huart1) {
//	        // Handle received data (if needed)
//	        // Here, we're simply sending back "OK"
//	        //HAL_UART_Transmit(&huart1, (uint8_t *)"OK", strlen("OK"), HAL_MAX_DELAY);
//
//	        // Start receiving again to wait for the next message
//	        HAL_UART_Receive_IT(&huart1, (uint8_t *)rx_buffer, 1);
//	}


//	if (huart->Instance == USART1) {
//			// Process the received message
//			//HAL_UART_Receive_IT(&huart1, USART1_RxBuffer, 10);
//		    if (strcmp((char*)USART1_RxBuffer, "<ENI>") == 0) {
//		        // If the received message is "<ENI>", send "OK"
//		        //sendResponse("OK");
//		    	HAL_UART_Transmit(&huart1, (uint8_t *)"<OK>", strlen("<OK>"), HAL_MAX_DELAY);
//		    	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)"ok", strlen("ok"));
//		    	//sendResponse("ok");
//
//
//		    	//HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data), HAL_MAX_DELAY);
//		    } else if (strcmp((char*)USART1_RxBuffer, "<MOV>") == 0){
//		        // If the received message is something else, send "NOK"
//		        //sendResponse("NOK");
//		    	//HAL_UART_Transmit_IT(&huart1, (uint8_t *)"nok", strlen("nok"));
//		    	HAL_UART_Transmit(&huart1, (uint8_t *)"<NOK>", strlen("<NOK>"), HAL_MAX_DELAY);
//		    	//sendResponse("nok");
//
//
//				//HAL_UART_Transmit(&huart1, (uint8_t *)tx_data, strlen(tx_data), HAL_MAX_DELAY);
//
//
//		    }
//
//		    // Restart reception for the next message
//		    //HAL_UART_Receive_IT(&huart3, USARTRxBuffer, 5);
//		    HAL_UART_Receive_IT(&huart1, USART1_RxBuffer, 10);
//		}

//	if (huart->Instance == USART1) {
//	        // Ensure the received buffer is null-terminated
//	        USART1_RxBuffer[10] = '\0';  // Assuming the buffer size is 10
//
//	        // Debugging: Print the received message
//	        printf("Received message: '%s'\n", (char*)USART1_RxBuffer);
//
//	        // Process the received message
//	        if (strcmp((char*)USART1_RxBuffer, "<ENI>") == 0) {
//	            // If the received message is "<ENI>", send "OK"
//	            HAL_UART_Transmit(&huart1, (uint8_t *)"<OK>", strlen("<OK>"), HAL_MAX_DELAY);
//	        } else {
//	            // If the received message is something else, send "NOK"
//	            HAL_UART_Transmit(&huart1, (uint8_t *)"<NOK>", strlen("<NOK>"), HAL_MAX_DELAY);
//	        }
//
//	        // Restart reception for the next message
//	        HAL_UART_Receive_IT(&huart1, USART1_RxBuffer, 10);
//	    }




	if (huart->Instance == USART1) {
	        // Ensure the received buffer is null-terminated
	        USART1_RxBuffer[5] = '\0';

	        // Debugging: Print the received message
	        printf("Received message: '%s'\n", (char*)USART1_RxBuffer);

	        // Process the received message
	        if (strcmp((char*)USART1_RxBuffer, "<IST>") == 0) {
	            HAL_UART_Transmit(&huart1, (uint8_t *)"<OK>", strlen("<OK>"), HAL_MAX_DELAY);
	        }
	        else if (strcmp((char*)USART1_RxBuffer, "<MOV>") == 0) {
	            HAL_UART_Transmit(&huart1, (uint8_t *)"<-0.19,-0.47,1.00,-93.65,-29.40,8.75,-116,-123,-1426,226.68,-133.32>", strlen("<-0.19,-0.47,1.00,-93.65,-29.40,8.75,-116,-123,-1426,226.68,-133.32>"), HAL_MAX_DELAY);
	            //<Accel_X,Accel_Y,Accel_Z,Gyro_X,Gyro_Y,Gyro_Z,Magn_Xaxis,Magn_Yaxis,Magn_Zaxis,Magn_heading,Magn_compas>
	        }
	        else if (strcmp((char*)USART1_RxBuffer, "<ENI>") == 0) {
	        	HAL_UART_Transmit(&huart1, (uint8_t *)"<27.3,101.2,52.9>", strlen("<27.3,101.2,52.9>"), HAL_MAX_DELAY);
	        	//<temperature,pressure,humidity>
	        }
	        else if (strcmp((char*)USART1_RxBuffer, "<ENE>") == 0) {
	        	HAL_UART_Transmit(&huart1, (uint8_t *)"<17.8,98.1,77.6>", strlen("<17.8,98.1,77.6>"), HAL_MAX_DELAY);
	        	//<temperature,pressure,humidity>
	        }
	        else if (strcmp((char*)USART1_RxBuffer, "<STR>") == 0) {
	        	HAL_UART_Transmit(&huart1, (uint8_t *)"<initiated>", strlen("<initiated>"), HAL_MAX_DELAY);
	        }
	        else {
	            HAL_UART_Transmit(&huart1, (uint8_t *)"<NOK>", strlen("<NOK>"), HAL_MAX_DELAY);
	        }

	        // Clear the buffer for the next reception
	        ClearBuffer(USART1_RxBuffer, 6);

	        // Restart reception for the next message
	        HAL_UART_Receive_IT(&huart1, USART1_RxBuffer, 5);
	    }





//	if (huart->Instance == USART1) {
//		// Check if the received command is "<ENI>"
//		if (strncmp((char*)huart->pRxBuffPtr, "<ENI>", 5) == 0) {
//			// Construct the string with sensor data
//			char dataString[100];
//			//BME280_struct sensorData; // Assuming sensorData is initialized elsewhere
//			sprintf(dataString, "<%s,%s,%s>", doubleToString(Sensors.BME280_Internal.Temperature),
//	                                              doubleToString(Sensors.BME280_Internal.Pressure),
//	                                              doubleToString(Sensors.BME280_Internal.Humidity));
//
//			// Transmit the string through UART1
//			HAL_UART_Transmit(&huart1, (uint8_t*)dataString, strlen(dataString), HAL_MAX_DELAY);
//	        }
//	    }






	/*
	if (huart == &huart1)
	{
		myCnt[2]++;
		HAL_UART_Receive_IT(&huart1, Rx_command1, 1);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	}
	if (huart == &huart3)
	{
		//myCnt[3]++;
		HAL_UART_Receive_IT(&huart3, Rx_command3, 1);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		if(Rx_command3[0] == 49){
			myCnt[4]++;
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
		}
		else if(Rx_command3[0] == 50){
					myCnt[4]++;
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
				}
	}
	*/

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_0) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
  } else {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
