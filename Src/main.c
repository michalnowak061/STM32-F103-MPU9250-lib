/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu9250.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t I_Time_Start = 0;
uint32_t I_Time_Stop = 0;
float dt = 0;

struct MPU9250 mpu1;

uint8_t MPU9250_status = 0;

int16_t Acce_X_global = 0, Acce_Y_global = 0, Acce_Z_global = 0;
int16_t Gyro_X_global = 0, Gyro_Y_global = 0, Gyro_Z_global = 0;
int16_t Mag_X_global  = 0, Mag_Y_global  = 0, Mag_Z_global  = 0;

float Acce_X_offset_global = 0, Acce_Y_offset_global = 0, Acce_Z_offset_global = 0;
float Gyro_X_offset_global = 0 , Gyro_Y_offset_global = 0, Gyro_Z_offset_global = 0;
float Mag_X_offset_global = 0, Mag_Y_offset_global = 0, Mag_Z_offset_global = 0;

float Acce_X_g_global = 0, Acce_Y_g_global = 0, Acce_Z_g_global = 0;
float Gyro_X_dgs_global = 0, Gyro_Y_dgs_global = 0, Gyro_Z_dgs_global = 0;
float Mag_X_uT_global = 0, Mag_Y_uT_global = 0, Mag_Z_uT_global = 0;

float Acce_Roll_global = 0, Acce_Pitch_global = 0;
float Gyro_Roll_global = 0, Gyro_Pitch_global = 0, Gyro_Yaw_global = 0;
float Mag_Yaw_global = 0;

float Acce_AlphaBeta_Roll_global = 0, Acce_AlphaBeta_Pitch_global = 0;

float Complementary_Roll = 0, Complementary_Pitch = 0, Complementary_Yaw = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);

  if( MPU9250_Init(&hi2c1, &mpu1, MPU9250_Device_1, MPU9250_Acce_2G, MPU9250_Gyro_2000s) == MPU9250_Init_OK) {

	  /*
	  if( MPU9250_Calibration(&hi2c1, &mpu1) == MPU9250_Calib_OK ) {

		  for(int i = 0; i < 3; ++i) {

			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			  HAL_Delay(200);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  HAL_Delay(200);
		  }
	  }
	  */

	  if( MPU9250_Set_Offsets(&hi2c1, &mpu1,
			  	  	  	  20.656, -167.292, 274.652,
						  -0.836, -3.57, 14.693,
						  24, 146, -92.5) == MPU9250_Offset_OK ) {

		  for(int i = 0; i < 3; ++i) {

			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  	  HAL_Delay(200);
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
 			  HAL_Delay(200);
  		  }
	  }

	  MPU9250_status = 1;
  }
  else {

	  MPU9250_status = 0;
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }

  HAL_Delay(100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  I_Time_Start = I_Time_Stop;
	  I_Time_Stop = HAL_GetTick();
	  dt = (float)( I_Time_Stop - I_Time_Start ) / 1000;

	  if (MPU9250_status == 1) {

		  MPU9250_Calculate_RPY(&hi2c1, &mpu1, dt);

		  AlphaBeta_filter(&mpu1, 0.2, 0.05, 0, 0, 0, 0, dt);
		  Complementary_filter(&mpu1, 0.02, dt);

		  Acce_X_offset_global = mpu1.Accelerometer_X_offset, Acce_Y_offset_global = mpu1.Accelerometer_Y_offset, Acce_Z_offset_global = mpu1.Accelerometer_Z_offset;
		  Gyro_X_offset_global = mpu1.Gyroscope_X_offset , Gyro_Y_offset_global = mpu1.Gyroscope_Y_offset, Gyro_Z_offset_global = mpu1.Gyroscope_Z_offset;
		  Mag_X_offset_global = mpu1.Magnetometer_X_offset, Mag_Y_offset_global = mpu1.Magnetometer_Y_offset, Mag_Z_offset_global = mpu1.Magnetometer_Z_offset;

		  Acce_X_global = mpu1.Accelerometer_X; Acce_Y_global = mpu1.Accelerometer_Y; Acce_Z_global = mpu1.Accelerometer_Z;
		  Gyro_X_global = mpu1.Gyroscope_X; Gyro_Y_global = mpu1.Gyroscope_Y; Gyro_Z_global = mpu1.Gyroscope_Z;
		  Mag_X_global  = mpu1.Magnetometer_X; Mag_Y_global  = mpu1.Magnetometer_Y; Mag_Z_global  = mpu1.Magnetometer_Z;

		  Acce_X_g_global = mpu1.Accelerometer_X_g; Acce_Y_g_global = mpu1.Accelerometer_Y_g; Acce_Z_g_global = mpu1.Accelerometer_Z_g;
		  Gyro_X_dgs_global = mpu1.Gyroscope_X_dgs; Gyro_Y_dgs_global = mpu1.Gyroscope_Y_dgs; Gyro_Z_dgs_global = mpu1.Gyroscope_Z_dgs;
		  Mag_X_uT_global = mpu1.Magnetometer_X_uT; Mag_Y_uT_global = mpu1.Magnetometer_Y_uT; Mag_Z_uT_global = mpu1.Magnetometer_Z_uT;

		  Acce_Roll_global = mpu1.Accelerometer_Roll, Acce_Pitch_global = mpu1.Accelerometer_Pitch;
		  Gyro_Roll_global = mpu1.Gyroscope_Roll, Gyro_Pitch_global = mpu1.Gyroscope_Pitch, Gyro_Yaw_global = mpu1.Gyroscope_Yaw;
		  Mag_Yaw_global = mpu1.Magnetometer_Yaw;

		  Acce_AlphaBeta_Roll_global = mpu1.Acce_AlphaBeta_Roll;

		  Complementary_Roll	= mpu1.Complementary_filter_Roll,
		  Complementary_Pitch	= mpu1.Complementary_filter_Pitch,
		  Complementary_Yaw 	= mpu1.Complementary_filter_Yaw;
	  }

	  HAL_Delay(10);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
