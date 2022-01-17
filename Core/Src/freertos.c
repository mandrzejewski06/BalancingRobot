/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "LSM6.h"
#include "printf.h"
#include "i2c.h"
#include "usart.h"
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

/* USER CODE END Variables */
/* Definitions for Heartbeat */
osThreadId_t HeartbeatHandle;
const osThreadAttr_t Heartbeat_attributes = {
  .name = "Heartbeat",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for MutexI2C */
osMutexId_t MutexI2CHandle;
const osMutexAttr_t MutexI2C_attributes = {
  .name = "MutexI2C"
};
/* Definitions for SemaphoreLSM6_DataReady */
osSemaphoreId_t SemaphoreLSM6_DataReadyHandle;
const osSemaphoreAttr_t SemaphoreLSM6_DataReady_attributes = {
  .name = "SemaphoreLSM6_DataReady"
};
/* Definitions for SemaphorePrint */
osSemaphoreId_t SemaphorePrintHandle;
const osSemaphoreAttr_t SemaphorePrint_attributes = {
  .name = "SemaphorePrint"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartHeartbeatTask(void *argument);
void StartIMU_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexI2C */
  MutexI2CHandle = osMutexNew(&MutexI2C_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreLSM6_DataReady */
  SemaphoreLSM6_DataReadyHandle = osSemaphoreNew(1, 1, &SemaphoreLSM6_DataReady_attributes);

  /* creation of SemaphorePrint */
  SemaphorePrintHandle = osSemaphoreNew(1, 1, &SemaphorePrint_attributes);

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
  /* creation of Heartbeat */
  HeartbeatHandle = osThreadNew(StartHeartbeatTask, NULL, &Heartbeat_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(StartIMU_Task, NULL, &IMU_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
  * @brief  Function implementing the Heartbeat thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartbeatTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      osDelay(500);
  }
  /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartIMU_Task */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU_Task */
void StartIMU_Task(void *argument)
{
  /* USER CODE BEGIN StartIMU_Task */
	LSM6_t LSM6DS33;
	osMutexAcquire(MutexI2CHandle, osWaitForever);
	if (LSM6_InitEx(&hi2c1, &LSM6DS33, device_autoDetect, sa0_autoDetect) == false)
	{
		printf("LSM6 INIT ERROR\n\r");
	}
	else
	{
		enableDefault(&LSM6DS33);
	}
	osMutexRelease(MutexI2CHandle);

  /* Infinite loop */
  for(;;)
  {
    if(osOK == osSemaphoreAcquire(SemaphoreLSM6_DataReadyHandle, osWaitForever))
    {
    	LSM6_Read(&LSM6DS33);
    	printf("ACC: x=%d, ", (int) LSM6DS33.accelerometer.x);
    	printf("y=%d, ", (int) LSM6DS33.accelerometer.y);
    	printf("z=%d, ", (int) LSM6DS33.accelerometer.z);
    	printf("GYRO: x=%d, ", (int) LSM6DS33.gyroscope.x);
    	printf("y=%d, ", (int) LSM6DS33.gyroscope.x);
    	printf("z=%d\n\r", (int) LSM6DS33.gyroscope.x);
    }
  }
  /* USER CODE END StartIMU_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
  // send char to console etc.
	osSemaphoreAcquire(SemaphorePrintHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 1000);
	osSemaphoreRelease(SemaphorePrintHandle);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LSM6_Interrupt_Pin)
	{
		osSemaphoreRelease(SemaphoreLSM6_DataReadyHandle);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
