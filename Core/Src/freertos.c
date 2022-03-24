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
#include "DRV8834.h"
#include "tim.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LSM6_SAMPLE_TIME 52 // Hz
#define GYRO_SENSITIVITY	0.00875		// at FS = +-245dps
#define ACC_SENSITIVITY		0.000061	// at FS = +-2g
#define GYRO_OFFSET_X 	-2.00
#define ACC_OFFSET_Y	0.05
#define ACC_OFFSET_Z	-0.02

#define PID_SAMPLE_TIME 52 // Hz
#define PID_SETPOINT 0.0

#define PID_DATA_READY 0x00000001
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const double COMP_FILTER_GAIN = 0.96;
double pidMonitorInput;
double pidMonitorOutput;
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
/* Definitions for MutexMotors */
osMutexId_t MutexMotorsHandle;
const osMutexAttr_t MutexMotors_attributes = {
  .name = "MutexMotors"
};
/* Definitions for SemaphoreLSM6_DataReady */
osSemaphoreId_t SemaphoreLSM6_DataReadyHandle;
const osSemaphoreAttr_t SemaphoreLSM6_DataReady_attributes = {
  .name = "SemaphoreLSM6_DataReady"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
double calculateAccAngle(double accY, double accZ);
double complementaryFilter(double gyroX, double accAngle, double prevAngle);
uint32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
/* USER CODE END FunctionPrototypes */

void StartHeartbeatTask(void *argument);
void StartIMU_Task(void *argument);
void StartPIDTask(void *argument);
void StartStepperMotorsTask(void *argument);

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

  /* creation of MutexMotors */
  MutexMotorsHandle = osMutexNew(&MutexMotors_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreLSM6_DataReady */
  SemaphoreLSM6_DataReadyHandle = osSemaphoreNew(1, 1, &SemaphoreLSM6_DataReady_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

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
	  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
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
	osDelay(150);
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
    	osMutexAcquire(MutexI2CHandle, osWaitForever);
    	if(HAL_OK == LSM6_Read(&LSM6DS33))
    	{
    		osMutexRelease(MutexI2CHandle);
        	accAngle = calculateAccAngle(LSM6DS33.accelerometer.y, LSM6DS33.accelerometer.z);
        	outputAngle = complementaryFilter(LSM6DS33.gyroscope.x, accAngle, prevAngle);
        	osMessageQueuePut(QueueInputPIDHandle, &outputAngle, 0, 0);
        	prevAngle = outputAngle;
        	printf("aY = %.2f, ", LSM6DS33.accelerometer.y*ACC_SENSITIVITY);
        	printf("aZ = %.2f, ", LSM6DS33.accelerometer.z*ACC_SENSITIVITY);
        	printf("gX = %.2f, ", LSM6DS33.gyroscope.x*GYRO_SENSITIVITY);
        	printf("accAngle = %.2f, ", accAngle);
        	printf("comp filter = %.2f\n\r", outputAngle);
    	}
    	else
    	{
    		osMutexRelease(MutexI2CHandle);
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
	double Kp = 3.0;
//	char KpTemp[5];
	double Ki = 14.0;
	double Kd = 0.015;

	PID_Init(&PID, &input, &output, &setpoint, Kp, Ki, Kd, P_ON_ERROR, DIRECT);
	PID_SetMode(&PID, AUTOMATIC);
	PID_SetSampleTime(&PID, (uint32_t) ((1.0/(float)PID_SAMPLE_TIME) * 1000.0));
	PID_SetOutputLimits(&PID, -DRV8834_MAX_SPEED, DRV8834_MAX_SPEED);
//	osTimerStart(TimerPIDHandle, (PID.SampleTime * osKernelGetTickFreq()) / 1000);
  /* Infinite loop */
  for(;;)
  {
	  //osSemaphoreAcquire(SemaphorePIDComputeHandle, osWaitForever);
	  osMessageQueueGet(QueueInputPIDHandle, &input, NULL, osWaitForever);
//	  if(HAL_OK == HAL_UART_Receive(&huart2, (uint8_t*)KpTemp, sizeof(KpTemp), 2))
//	  {
//		  Kp = atof(KpTemp);
//	  }
//	  printf("Kp: %.2f\n\r", Kp);
	  pidMonitorInput = input;
	  if(input < 40 && input >-40)
	  {
		  PID_Compute(&PID);
	  }
	  else
	  {
		  output = 0.0;
	  }

	  if(output < 5 && output > -5)
	  {
		  output = 0.0;
	  }
	  osMessageQueuePut(QueueOutputPIDHandle, &output, 0, 0);
	  pidMonitorOutput = output;
	  printf("PID input: %.2f, output:%.2f\n\r", input, output);
	  //osThreadFlagsSet(StepperMotorsTaHandle, PID_DATA_READY);
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
	StepMotor_t rightMotor;
	StepMotor_t leftMotor;
	double pid_output;
	int32_t speed = 0;

	DRV8834_Init(&leftMotor, &htim2, TIM_CHANNEL_3);
	DRV8834_InitPins(&leftMotor, leftMotor_DIR_Pin, leftMotor_DIR_GPIO_Port,
			leftMotor_M0_Pin, leftMotor_M0_GPIO_Port, leftMotor_M1_Pin, leftMotor_M1_GPIO_Port);
	DRV8834_SetMicrostep(&leftMotor, 4);

	DRV8834_Init(&rightMotor, &htim4, TIM_CHANNEL_1);
	DRV8834_InitPins(&rightMotor, rightMotor_DIR_Pin, rightMotor_DIR_GPIO_Port,
			rightMotor_M0_Pin, rightMotor_M0_GPIO_Port, rightMotor_M1_Pin, rightMotor_M1_GPIO_Port);
	DRV8834_SetMicrostep(&rightMotor, 4);

//	DRV8834_StartMotor(&leftMotor, 1);
//	DRV8834_StartMotor(&rightMotor, 1);
  /* Infinite loop */
  for(;;)
  {
	//osThreadFlagsWait(PID_DATA_READY, osFlagsWaitAll, osWaitForever);
    if(osOK == osMessageQueueGet(QueueOutputPIDHandle, &pid_output, NULL, osWaitForever))
    {
    	speed = (int32_t) pid_output;
    	printf("Step motor input speed:%d\n\r", speed);
    	if((leftMotor.state == STOPPED) && (rightMotor.state == STOPPED) && (speed != 0))
    	{
    		osMutexAcquire(MutexMotorsHandle, osWaitForever);
    		DRV8834_StartMotor(&leftMotor, -speed);
    		DRV8834_StartMotor(&rightMotor, speed);
    		osMutexRelease(MutexMotorsHandle);
    	}
    	else
    	{
    		osMutexAcquire(MutexMotorsHandle, osWaitForever);
        	DRV8834_SetSpeed(&leftMotor, -speed);
        	DRV8834_SetSpeed(&rightMotor, speed);
    		osMutexRelease(MutexMotorsHandle);
    	}
    }
//	      	DRV8834_SetSpeed(&leftMotor, 1);
//	      	DRV8834_SetSpeed(&rightMotor, 1);
//	      	osDelay(500);
  }
  /* USER CODE END StartStepperMotorsTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void _putchar(char character)
{
  // send char to console etc.
	osMutexAcquire(MutexUARTputcharHandle, osWaitForever);
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 10);
	osMutexRelease(MutexUARTputcharHandle);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LSM6_Interrupt_Pin)
	{
		osSemaphoreRelease(SemaphoreLSM6_DataReadyHandle);
	}
}

uint32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
