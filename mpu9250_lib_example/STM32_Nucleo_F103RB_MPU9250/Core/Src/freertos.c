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

#include "stdio.h"

//#include "stdio.h"

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
/* USER CODE BEGIN Variables */

/* ------------> MPU9250 variables <------------- */
float a_x_offset_global = 0, a_y_offset_global = 0, a_z_offset_global = 0;
float g_x_offset_global = 0, g_y_offset_global = 0, g_z_offset_global = 0;
float m_x_offset_global = 0, m_y_offset_global = 0, m_z_offset_global = 0;
float m_x_scale_global = 0, m_y_scale_global = 0, m_z_scale_global = 0;

float a_g_x = 0, a_g_y = 0, a_g_z = 0;
float a_g_offset_x = 0, a_g_offset_y = 0, a_g_offset_z = 0;
float a_wihout_g_x = 0, a_wihout_g_y = 0, a_wihout_g_z = 0;
float a_velocity_x = 0, a_velocity_y = 0, a_velocity_z = 0;
float a_position_x = 0, a_position_y = 0, a_position_z = 0;

float Madgwick_q_w = 1, Madgwick_q_x = 0, Madgwick_q_y = 0, Madgwick_q_z = 0;

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
void Start_USART_Task(void const * argument)
{
  /* USER CODE BEGIN Start_USART_Task */

	/* Start receiving */
	HAL_UART_Receive_DMA(HC05_handle, Data_from_PC, DATA_FRAME_FROM_PC_SIZE);

	/* Infinite loop */
	for (;;) {

		HC05_Fill_Data_frame_to_PC(&DT_PC, Data_to_PC,
									a_position_x, a_position_y, a_position_z, 0,
									0, 0, 0, 0,
									/*a_position_x, a_position_y, a_position_z, 0*/
									Madgwick_q_w, Madgwick_q_x, Madgwick_q_y, Madgwick_q_z);

		HAL_UART_Transmit_DMA(HC05_handle, Data_to_PC, DATA_FRAME_TO_PC_SIZE);

		uint16_t cnt[10];
		uint8_t  data[50];
		uint16_t size = 0;

		cnt[0] = (uint16_t)(a_wihout_g_x * 1000);
		cnt[1] = (uint16_t)(a_wihout_g_y * 1000);
		cnt[2] = (uint16_t)(a_wihout_g_z * 1000);

		cnt[3] = (uint16_t)(a_velocity_x * 1000);
		cnt[4] = (uint16_t)(a_velocity_y * 1000);
		cnt[5] = (uint16_t)(a_velocity_z * 1000);

		cnt[6] = (uint16_t)(a_position_x * 1000);
		cnt[7] = (uint16_t)(a_position_y * 1000);
		cnt[8] = (uint16_t)(a_position_z * 1000);


		size = sprintf(data, "a_wihout_g_x: %d \n\r", cnt[0]);
		HAL_UART_Transmit(&huart2, data, size, 10);
		size = sprintf(data, "a_wihout_g_y: %d \n\r", cnt[1]);
		HAL_UART_Transmit(&huart2, data, size, 10);
		size = sprintf(data, "a_wihout_g_z: %d \n\r", cnt[2]);
		HAL_UART_Transmit(&huart2, data, size, 10);

		size = sprintf(data, "a_velocity_x: %d \n\r", cnt[3]);
		HAL_UART_Transmit(&huart2, data, size, 10);
		size = sprintf(data, "a_velocity_y: %d \n\r", cnt[4]);
		HAL_UART_Transmit(&huart2, data, size, 10);
		size = sprintf(data, "a_velocity_z: %d \n\r", cnt[5]);
		HAL_UART_Transmit(&huart2, data, size, 10);

		size = sprintf(data, "a_position_x: %d \n\r", cnt[6]);
		HAL_UART_Transmit(&huart2, data, size, 10);
		size = sprintf(data, "a_position_y: %d \n\r", cnt[7]);
		HAL_UART_Transmit(&huart2, data, size, 10);
		size = sprintf(data, "a_position_z: %d \n\r", cnt[8]);
		HAL_UART_Transmit(&huart2, data, size, 10);

		size = sprintf(data, "\n\r", cnt[9]);
		HAL_UART_Transmit(&huart2, data, size, 10);

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
void Start_IMU_Task(void const * argument)
{
  /* USER CODE BEGIN Start_IMU_Task */

	/* IMU task variables */
	uint8_t mpu9250_correct_init_global = 0;

	struct MPU9250 mpu1;

	uint32_t I_Time_Stop = 0;
	uint32_t I_Time_Start = 0;

	/* IMU task initialization */
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	osDelay(100);

	if (MPU9250_Init(&hi2c1, &mpu1, MPU9250_Device_1, MPU9250_Acce_2G, MPU9250_Gyro_2000s) == MPU9250_Init_OK) {

		MPU9250_Set_Offsets(&hi2c1, &mpu1, 0,0,0, 0,0,0, -26.536,0.992,0.968, 1.04,1,1);


		MPU9250_Calibration_Acce(&hi2c1, &mpu1);
		MPU9250_Calibration_Gyro(&hi2c1, &mpu1);
		//MPU9250_Calibration_Mag(&hi2c1, &mpu1);

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

	/* Infinite loop */
	for (;;) {

		if (mpu9250_correct_init_global == 1) {

			/* Case 1: Time measurement */
			I_Time_Start = I_Time_Stop;
			I_Time_Stop = HAL_GetTick();

			dt = (float) (I_Time_Stop - I_Time_Start) / 1000;

			/* Case 2: Update AHRS */
			MPU9250_Update(&hi2c1, &mpu1, dt);

			a_g_x = mpu1.Accelerometer_vector_g.x,
			a_g_y = mpu1.Accelerometer_vector_g.y,
			a_g_z = mpu1.Accelerometer_vector_g.z;

			a_g_offset_x = mpu1.Accelerometer_vector_g_offset.x,
			a_g_offset_y = mpu1.Accelerometer_vector_g_offset.y,
			a_g_offset_z = mpu1.Accelerometer_vector_g_offset.z;

			a_wihout_g_x = mpu1.Accelerometer_vector_without_g.x,
			a_wihout_g_y = mpu1.Accelerometer_vector_without_g.y,
			a_wihout_g_z = mpu1.Accelerometer_vector_without_g.z;

			a_velocity_x = mpu1.Accelerometer_vector_velocity.x,
			a_velocity_y = mpu1.Accelerometer_vector_velocity.y,
			a_velocity_z = mpu1.Accelerometer_vector_velocity.z;

			a_position_x = mpu1.Accelerometer_vector_position.x,
			a_position_y = mpu1.Accelerometer_vector_position.y,
			a_position_z = mpu1.Accelerometer_vector_position.z;

			/* Filters data */
			Madgwick_q_w = mpu1.Madgwick_quaternion.w;
			Madgwick_q_x = mpu1.Madgwick_quaternion.x;
			Madgwick_q_y = mpu1.Madgwick_quaternion.y;
			Madgwick_q_z = mpu1.Madgwick_quaternion.z;
		}

		osDelay(10);
	}
  /* USER CODE END Start_IMU_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/*
int _write(int file , char *ptr , int len) {

	HAL_UART_Transmit(&huart2 ,(uint8_t *)ptr ,len ,1000) ;
	return len;
}
*/
     
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
