/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "mpu9250.h"
#include "hc05.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMPLEMENTARY_FILTER_WEIGHT 0.01
#define KALMAN_PROCESS_VARIANCE		0.001
#define KALMAN_MEASURE_VARIANCE		0.1
#define MADGWICK_BETA				0.01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ------------> MPU9250 variables <------------- */
float a_x_g_global = 0, a_y_g_global = 0, a_z_g_global = 0;
float g_x_dgs_global = 0, g_y_dgs_global = 0, g_z_dgs_global = 0;
float m_x_uT_global = 0, m_y_uT_global = 0, m_z_uT_global = 0;

float a_roll_global = 0, a_pitch_global = 0;
float g_roll_global  = 0, g_pitch_global = 0, g_yaw_global   = 0;
float m_yaw_global = 0;

float a_x_offset_global = 0, a_y_offset_global = 0, a_z_offset_global = 0;
float g_x_offset_global = 0, g_y_offset_global = 0, g_z_offset_global = 0;
float m_x_offset_global = 0, m_y_offset_global = 0, m_z_offset_global = 0;
float m_x_scale_global = 0, m_y_scale_global = 0, m_z_scale_global = 0;

float Complementary_Roll_global = 0, Complementary_Pitch_global = 0, Complementary_Yaw_global = 0;
float Kalman_Roll_global = 0, Kalman_Pitch_global = 0, Kalman_Yaw_global = 0;
float Madgwick_Roll_global = 0, Madgwick_Pitch_global = 0, Madgwick_Yaw_global = 0;

float Filter_weight_global = COMPLEMENTARY_FILTER_WEIGHT;
float Kalman_filter_process_variance = KALMAN_PROCESS_VARIANCE;
float Kalman_filter_measure_variance = KALMAN_MEASURE_VARIANCE;
float Madgwick_filter_beta = MADGWICK_BETA;

float dt = 0;

/* -----------> Communication variables <----------- */
UART_HandleTypeDef *HC05_handle = &huart1;

struct Data_frame_from_PC	DF_PC;
struct Data_frame_to_PC		DT_PC;

uint8_t Data_to_PC[DATA_FRAME_TO_PC_SIZE];
uint8_t Data_from_PC[DATA_FRAME_FROM_PC_SIZE];

/* -----------> Additional variables    <----------- */
uint8_t Which_filter_global = 0;

/* USER CODE END Variables */
osThreadId USART_TaskHandle;
osThreadId IMU_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void Start_USART_Task(void const * argument);
void Start_IMU_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of USART_Task */
  osThreadDef(USART_Task, Start_USART_Task, osPriorityNormal, 0, 512);
  USART_TaskHandle = osThreadCreate(osThread(USART_Task), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, Start_IMU_Task, osPriorityRealtime, 0, 512);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_USART_Task */
/**
* @brief Function implementing the USART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_USART_Task */
void Start_USART_Task(void const *argument) {
	/* USER CODE BEGIN Start_USART_Task */

	/* Start receiving */
	HAL_UART_Receive_DMA(HC05_handle, Data_from_PC, DATA_FRAME_FROM_PC_SIZE);

	/* Infinite loop */
	for (;;) {

		HC05_Fill_Data_frame_to_PC(&DT_PC, Data_to_PC,
				Complementary_Roll_global, Complementary_Pitch_global, Complementary_Yaw_global,
				Kalman_Roll_global, Kalman_Pitch_global, Kalman_Yaw_global,
				Madgwick_Roll_global, Madgwick_Pitch_global, Madgwick_Yaw_global);

		HAL_UART_Transmit_DMA(HC05_handle, Data_to_PC, DATA_FRAME_TO_PC_SIZE);

		osDelay(10);
	}

	/* USER CODE END Start_USART_Task */
}

/* USER CODE BEGIN Header_Start_IMU_Task */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_IMU_Task */
void Start_IMU_Task(void const *argument) {
	/* USER CODE BEGIN Start_IMU_Task */

	/* IMU task variables */
	uint8_t mpu9250_correct_init_global = 0;

	struct MPU9250 mpu1;

	uint32_t I_Time_Stop = 0;
	uint32_t I_Time_Start = 0;

	/* IMU task initialization */
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	osDelay(100);

	if (MPU9250_Init(&hi2c1, &mpu1, MPU9250_Device_1, MPU9250_Acce_2G,
			MPU9250_Gyro_2000s) == MPU9250_Init_OK) {

		//MPU9250_Set_Offsets(&hi2c1, &mpu1, 0,0,0, 0,0,0, 27.5,0.97,0.95, 1.07,1,1);

		MPU9250_Calibration_Acce(&hi2c1, &mpu1);
		MPU9250_Calibration_Gyro(&hi2c1, &mpu1);
		MPU9250_Calibration_Mag(&hi2c1, &mpu1);

		a_x_offset_global = mpu1.Accelerometer_X_offset, a_y_offset_global = mpu1.Accelerometer_Y_offset, a_z_offset_global = mpu1.Accelerometer_Z_offset;
		g_x_offset_global = mpu1.Gyroscope_X_offset, g_y_offset_global = mpu1.Gyroscope_Y_offset, g_z_offset_global = mpu1.Gyroscope_Z_offset;
		m_x_offset_global = mpu1.Magnetometer_X_offset, m_y_offset_global = mpu1.Magnetometer_Y_offset, m_z_offset_global = mpu1.Magnetometer_Z_offset;
		m_x_scale_global = mpu1.Magnetometer_X_scale, m_y_offset_global = mpu1.Magnetometer_Y_scale, m_z_offset_global = mpu1.Magnetometer_Z_scale;

		for (int i = 0; i < 3; ++i) {

			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			osDelay(200);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			osDelay(200);
		}

		mpu9250_correct_init_global = 1;
	} else {

		mpu9250_correct_init_global = 0;
	}

	osDelay(100);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET) {

		osDelay(10);
	}

	//MPU9250_Calculate_RPY(&hi2c1, &mpu1, dt);
	//mpu1.Gyroscope_Roll  = mpu1.Accelerometer_Roll;
	//mpu1.Gyroscope_Pitch = mpu1.Accelerometer_Pitch;
	//mpu1.Gyroscope_Yaw   = mpu1.Magnetometer_Yaw;

	for(int i = 0; i < 1000; ++i) {

		MPU9250_Calculate_RPY(&hi2c1, &mpu1, dt);
		mpu1.Magnetometer_Yaw_offset += mpu1.Magnetometer_Yaw;
	}
	mpu1.Magnetometer_Yaw_offset = mpu1.Magnetometer_Yaw_offset / 1000;

	/* Infinite loop */
	for (;;) {
		if (mpu9250_correct_init_global == 1) {

			/* Case 1: Time measurement */
			I_Time_Start = I_Time_Stop;
			I_Time_Stop = HAL_GetTick();

			dt = (float) (I_Time_Stop - I_Time_Start) / 1000;

			/* Case 2: RPY calculation */
			MPU9250_Calculate_RPY(&hi2c1, &mpu1, dt);

			a_x_g_global = mpu1.Accelerometer_X_g, a_y_g_global = mpu1.Accelerometer_Y_g, a_z_g_global = mpu1.Accelerometer_Z_g;
			g_x_dgs_global = mpu1.Gyroscope_X_dgs, g_y_dgs_global = mpu1.Gyroscope_Y_dgs, g_z_dgs_global = mpu1.Gyroscope_Z_dgs;
			m_x_uT_global = mpu1.Magnetometer_X_uT, m_y_uT_global = mpu1.Magnetometer_Y_uT, m_z_uT_global = mpu1.Magnetometer_Z_uT;

			a_roll_global = mpu1.Accelerometer_Roll, a_pitch_global = mpu1.Accelerometer_Pitch;
			g_roll_global = mpu1.Gyroscope_Roll, g_pitch_global = mpu1.Gyroscope_Pitch, g_yaw_global = mpu1.Gyroscope_Yaw;
			m_yaw_global = mpu1.Magnetometer_Yaw;

			/* Case 3: Filters using */
			Complementary_filter(&mpu1, Filter_weight_global, dt);
			Kalman_filter(&mpu1, Kalman_filter_process_variance, Kalman_filter_measure_variance, dt);
			Madgwick_filter(&mpu1, Madgwick_filter_beta, dt);
			Mahony_filter(&mpu1, dt);

			/* Filters data */
			/*
			switch (Which_filter_global) {

				case 0:
					Filter_Roll_global  = mpu1.Complementary_filter_Roll;
					Filter_Pitch_global = mpu1.Complementary_filter_Pitch;
					Filter_Yaw_global   = mpu1.Complementary_filter_Yaw - mpu1.Magnetometer_Yaw_offset;
					break;

				case 1:
					Filter_Roll_global  = mpu1.Kalman_filter_Roll;
					Filter_Pitch_global = mpu1.Kalman_filter_Pitch;
					Filter_Yaw_global   = mpu1.Kalman_filter_Yaw - mpu1.Magnetometer_Yaw_offset;
					break;

				case 2:
					Filter_Roll_global  = mpu1.Madgwick_filter_Roll;
					Filter_Pitch_global = mpu1.Madgwick_filter_Pitch;
					Filter_Yaw_global   = mpu1.Madgwick_filter_Yaw - mpu1.Magnetometer_Yaw_offset;
					break;

				case 3:
					Filter_Roll_global  = mpu1.Mahony_filter_Roll;
					Filter_Pitch_global = mpu1.Mahony_filter_Pitch;
					Filter_Yaw_global   = mpu1.Mahony_filter_Yaw - mpu1.Magnetometer_Yaw_offset;
					break;
			}
			*/
			Complementary_Roll_global  = mpu1.Complementary_filter_Roll;
			Complementary_Pitch_global = mpu1.Complementary_filter_Pitch;
			Complementary_Yaw_global   = mpu1.Complementary_filter_Yaw - mpu1.Magnetometer_Yaw_offset;

			Kalman_Roll_global  = mpu1.Kalman_filter_Roll;
			Kalman_Pitch_global = mpu1.Kalman_filter_Pitch;
			Kalman_Yaw_global   = mpu1.Kalman_filter_Yaw - mpu1.Magnetometer_Yaw_offset;

			Madgwick_Roll_global  = mpu1.Madgwick_filter_Roll;
			Madgwick_Pitch_global = mpu1.Madgwick_filter_Pitch;
			Madgwick_Yaw_global   = mpu1.Madgwick_filter_Yaw - mpu1.Magnetometer_Yaw_offset;
		}

		osDelay(10);
	}
	/* USER CODE END Start_IMU_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {

		HAL_UART_Receive_DMA(HC05_handle, Data_from_PC, DATA_FRAME_FROM_PC_SIZE);

		if( HC05_Parse_Data_frame(&DF_PC, Data_from_PC) == 0 ) {

			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

			/* Filters data from PC */
			//Filter_weight_global           = (float) DF_PC.Complementary_filter_weight / 1000;

			//Kalman_filter_process_variance = DF_PC.Kalman_filter_process_variance;
			//Kalman_filter_measure_variance = DF_PC.Kalman_filter_measure_variance;

			//Madgwick_filter_beta           = (float) DF_PC.Madgwick_filter_beta / 1000;

			/* Additional data from PC */
			Which_filter_global = DF_PC.Which_filter;
		}
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
