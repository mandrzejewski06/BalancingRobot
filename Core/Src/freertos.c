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
#include "PID.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM6_SAMPLE_TIME 52 // Hz
#define GYRO_SENSITIVITY	0.00875		// at FS = +-245dps
#define ACC_SENSITIVITY		0.000061	// at FS = +-2g
#define GYRO_OFFSET_X 	-6.0
#define ACC_OFFSET_Y	-0.04
#define ACC_OFFSET_Z	-0.025

#define PID_SAMPLE_TIME 100 // Hz
#define PID_SETPOINT 0.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const double COMP_FILTER_GAIN = 0.96;
/* USER CODE END Variables */
/* Definitions for Heartbeat */
osThreadId_t HeartbeatHandle;
const osThreadAttr_t Heartbeat_attributes = {
  .name = "Heartbeat",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 256 * 4
};
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
  .name = "PIDTask",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for StepperMotorsTa */
osThreadId_t StepperMotorsTaHandle;
const osThreadAttr_t StepperMotorsTa_attributes = {
  .name = "StepperMotorsTa",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for QueueInputPID */
osMessageQueueId_t QueueInputPIDHandle;
const osMessageQueueAttr_t QueueInputPID_attributes = {
  .name = "QueueInputPID"
};
/* Definitions for QueueOutputPID */
osMessageQueueId_t QueueOutputPIDHandle;
const osMessageQueueAttr_t QueueOutputPID_attributes = {
  .name = "QueueOutputPID"
};
/* Definitions for TimerPID */
osTimerId_t TimerPIDHandle;
const osTimerAttr_t TimerPID_attributes = {
  .name = "TimerPID"
};
/* Definitions for MutexI2C */
osMutexId_t MutexI2CHandle;
const osMutexAttr_t MutexI2C_attributes = {
  .name = "MutexI2C"
};
/* Definitions for MutexUARTputchar */
osMutexId_t MutexUARTputcharHandle;
const osMutexAttr_t MutexUARTputchar_attributes = {
  .name = "MutexUARTputchar"
};
/* Definitions for SemaphoreLSM6_DataReady */
osSemaphoreId_t SemaphoreLSM6_DataReadyHandle;
const osSemaphoreAttr_t SemaphoreLSM6_DataReady_attributes = {
  .name = "SemaphoreLSM6_DataReady"
};
/* Definitions for SemaphorePIDCompute */
osSemaphoreId_t SemaphorePIDComputeHandle;
const osSemaphoreAttr_t SemaphorePIDCompute_attributes = {
  .name = "SemaphorePIDCompute"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
double calculateAccAngle(double accY, double accZ);
double complementaryFilter(double gyroX, double accAngle, double prevAngle);
/* USER CODE END FunctionPrototypes */

void StartHeartbeatTask(void *argument);
void StartIMU_Task(void *argument);
void StartPIDTask(void *argument);
void StartStepperMotorsTask(void *argument);
void TimerPIDCallback(void *argument);

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

  /* creation of MutexUARTputchar */
  MutexUARTputcharHandle = osMutexNew(&MutexUARTputchar_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreLSM6_DataReady */
  SemaphoreLSM6_DataReadyHandle = osSemaphoreNew(1, 1, &SemaphoreLSM6_DataReady_attributes);

  /* creation of SemaphorePIDCompute */
  SemaphorePIDComputeHandle = osSemaphoreNew(1, 1, &SemaphorePIDCompute_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerPID */
  TimerPIDHandle = osTimerNew(TimerPIDCallback, osTimerPeriodic, NULL, &TimerPID_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueInputPID */
  QueueInputPIDHandle = osMessageQueueNew (8, sizeof(double), &QueueInputPID_attributes);

  /* creation of QueueOutputPID */
  QueueOutputPIDHandle = osMessageQueueNew (16, sizeof(double), &QueueOutputPID_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Heartbeat */
  HeartbeatHandle = osThreadNew(StartHeartbeatTask, NULL, &Heartbeat_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(StartIMU_Task, NULL, &IMU_Task_attributes);

  /* creation of PIDTask */
  PIDTaskHandle = osThreadNew(StartPIDTask, NULL, &PIDTask_attributes);

  /* creation of StepperMotorsTa */
  StepperMotorsTaHandle = osThreadNew(StartStepperMotorsTask, NULL, &StepperMotorsTa_attributes);

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
	double accAngle, outputAngle;
	double prevAngle = 0;

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
    	if(HAL_OK == LSM6_Read(&LSM6DS33))
    	{
        	accAngle = calculateAccAngle(LSM6DS33.accelerometer.y, LSM6DS33.accelerometer.z);
        	outputAngle = complementaryFilter(LSM6DS33.gyroscope.x, accAngle, prevAngle);
        	osMessageQueuePut(QueueInputPIDHandle, &outputAngle, 0, 0);
        	prevAngle = outputAngle;
        	printf("accAngle = %.2f, ", accAngle);
        	printf("comp filter = %.2f\n\r", outputAngle);
    	}
    	else
    	{
    		printf("LSM6 READ ERROR\n\r");
    	}
    }
  }
  /* USER CODE END StartIMU_Task */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void *argument)
{
  /* USER CODE BEGIN StartPIDTask */
	PID_t PID;
	double input, output;
	double setpoint = PID_SETPOINT;
	double Kp = 21.0;
	double Ki = 240.0;
	double Kd = 0.8;

	PID_Init(&PID, &input, &output, &setpoint, Kp, Ki, Kd, P_ON_ERROR, DIRECT);
	PID_SetMode(&PID, AUTOMATIC);
	PID_SetSampleTime(&PID, (uint32_t) ((1.0/(float)PID_SAMPLE_TIME) * 1000.0));
	PID_SetOutputLimits(&PID, -128, 127);
	osTimerStart(TimerPIDHandle, (PID.SampleTime * osKernelGetTickFreq()) / 1000);
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(SemaphorePIDComputeHandle, osWaitForever);
	  osMessageQueueGet(QueueInputPIDHandle, &input, NULL, 0);
	  PID_Compute(&PID);
	  osMessageQueuePut(QueueOutputPIDHandle, &output, 0, 0);
	  printf("PID input: %.2f, output:%.2f\n\r", input, output);
  }
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartStepperMotorsTask */
/**
* @brief Function implementing the StepperMotorsTa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStepperMotorsTask */
void StartStepperMotorsTask(void *argument)
{
  /* USER CODE BEGIN StartStepperMotorsTask */
	double pid_output;
  /* Infinite loop */
  for(;;)
  {
    if(osOK == osMessageQueueGet(QueueOutputPIDHandle, &pid_output, NULL, 0))
    {
    	printf("Step motor input:%.2f\n\r", pid_output);
    }
    osDelay(5);
  }
  /* USER CODE END StartStepperMotorsTask */
}

/* TimerPIDCallback function */
void TimerPIDCallback(void *argument)
{
  /* USER CODE BEGIN TimerPIDCallback */
	osSemaphoreRelease(SemaphorePIDComputeHandle);
  /* USER CODE END TimerPIDCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
  // send char to console etc.
	osMutexAcquire(MutexUARTputcharHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 1000);
	osMutexRelease(MutexUARTputcharHandle);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LSM6_Interrupt_Pin)
	{
		osSemaphoreRelease(SemaphoreLSM6_DataReadyHandle);
	}
}

double complementaryFilter(double gyroX, double accAngle, double prevAngle)
{
	return (COMP_FILTER_GAIN * (prevAngle + ((gyroX*GYRO_SENSITIVITY + GYRO_OFFSET_X)/LSM6_SAMPLE_TIME)))
			+ ((1 - COMP_FILTER_GAIN) * accAngle);
}

double calculateAccAngle(double accY, double accZ)
{
	return (atan2((accY*ACC_SENSITIVITY + ACC_OFFSET_Y), (accZ*ACC_SENSITIVITY + ACC_OFFSET_Z)) * 180.0/M_PI);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
