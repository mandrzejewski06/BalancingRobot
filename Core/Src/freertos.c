/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LSM6.h"
#include "i2c.h"
#include "usart.h"
#include "PID.h"
#include "math.h"
#include "DRV8834.h"
#include "tim.h"
#include "stdlib.h"
#include "parser.h"
#include "ring_buffer.h"
#include "dma.h"
#include "stdio.h"
#include "string.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KERNEL_TICKS_FOR_MS 	1
#define TMP_DMAUART_BUFFER_SIZE 64

#define ROBOT_ZERO_HYSTERESIS 	5
#define ROBOT_MAX_ANGLE			40

#define IMU_SAMPLE_TIME 		52 			// Hz
#define IMU_GYRO_SENSITIVITY 	0.00875		// at FS = +-245dps
#define IMU_ACC_SENSITIVITY		0.000061	// at FS = +-2g
#define IMU_ACCY_DEFAULT_OFFSET	0.05
#define IMU_ACCZ_DEFAULT_OFFSET	-0.02
#define IMU_GYRX_DEFAULT_OFFSET	-2.0
#define IMU_AUTOCALIB_ITERATONS	100

#define PID_SAMPLE_TIME 	IMU_SAMPLE_TIME
#define PID_DEFAULT_SETPOINT 0.0
#define PID_DEFAULT_KP 		3.0
#define PID_DEFAULT_KI 		14.0
#define PID_DEFAULT_KD 		0.015

#define MOT_DEFAULT_MICROSTEP 4
#define MOT_TURNING_SPEED 5

#define OTH_BAT_NUM_OF_READINGS 10
#define OTH_BAT_DEFAULT_MIN_V	2.4
#define OTH_BAT_DEFAULT_MAX_V	3.4
#define OTH_ADC_MAX_VALUE		4096
#define OTH_ADC_VDDA_VALUE		3.3

#define PID_FLAGS 		0xFF
#define IMU_FLAGS 		0xFF
#define MOT_FLAGS 		0xFFF
#define OTH_FLAGS 		0xFFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CHECK_BIT(FLAG, BIT) ((BIT) == ((FLAG)&(BIT)))
#define ABS(value) (value < 0 ? -value : value)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const float COMP_FILTER_GAIN = 0.96;

uint8_t RxLines = 0;
float pidMonitorInput;
float pidMonitorOutput;
float pidMonitorSetpoint;

uint8_t Tmp_Rx_Buffer[TMP_DMAUART_BUFFER_SIZE];
uint8_t Tmp_Tx_Buffer[TMP_DMAUART_BUFFER_SIZE];

RingBuffer_t Rx_RingBuffer;
RingBuffer_t Tx_RingBuffer;

osSemaphoreId_t SemRxLineAvailableHandle;
const osSemaphoreAttr_t SemRxLineAvailable_attributes = {
  .name = "SemRxLineAvailable"
};
/* USER CODE END Variables */
/* Definitions for UARTReadTask */
osThreadId_t UARTReadTaskHandle;
const osThreadAttr_t UARTReadTask_attributes = {
  .name = "UARTReadTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IMU_Task */
osThreadId_t IMU_TaskHandle;
const osThreadAttr_t IMU_Task_attributes = {
  .name = "IMU_Task",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for PIDTask */
osThreadId_t PIDTaskHandle;
const osThreadAttr_t PIDTask_attributes = {
  .name = "PIDTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for StepperMotorsTa */
osThreadId_t StepperMotorsTaHandle;
const osThreadAttr_t StepperMotorsTa_attributes = {
  .name = "StepperMotorsTa",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OtherTask */
osThreadId_t OtherTaskHandle;
const osThreadAttr_t OtherTask_attributes = {
  .name = "OtherTask",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow4,
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
/* Definitions for QueueUartPID */
osMessageQueueId_t QueueUartPIDHandle;
const osMessageQueueAttr_t QueueUartPID_attributes = {
  .name = "QueueUartPID"
};
/* Definitions for QueueUartIMU */
osMessageQueueId_t QueueUartIMUHandle;
const osMessageQueueAttr_t QueueUartIMU_attributes = {
  .name = "QueueUartIMU"
};
/* Definitions for QueueUartMOT */
osMessageQueueId_t QueueUartMOTHandle;
const osMessageQueueAttr_t QueueUartMOT_attributes = {
  .name = "QueueUartMOT"
};
/* Definitions for QueueUartOTH */
osMessageQueueId_t QueueUartOTHHandle;
const osMessageQueueAttr_t QueueUartOTH_attributes = {
  .name = "QueueUartOTH"
};
/* Definitions for MutexI2C */
osMutexId_t MutexI2CHandle;
const osMutexAttr_t MutexI2C_attributes = {
  .name = "MutexI2C"
};
/* Definitions for MutexRingBufferRx */
osMutexId_t MutexRingBufferRxHandle;
const osMutexAttr_t MutexRingBufferRx_attributes = {
  .name = "MutexRingBufferRx"
};
/* Definitions for MutexRingBufferTx */
osMutexId_t MutexRingBufferTxHandle;
const osMutexAttr_t MutexRingBufferTx_attributes = {
  .name = "MutexRingBufferTx"
};
/* Definitions for SemaphoreLSM6_DataReady */
osSemaphoreId_t SemaphoreLSM6_DataReadyHandle;
const osSemaphoreAttr_t SemaphoreLSM6_DataReady_attributes = {
  .name = "SemaphoreLSM6_DataReady"
};
/* Definitions for SemaphoreADC_DataReady */
osSemaphoreId_t SemaphoreADC_DataReadyHandle;
const osSemaphoreAttr_t SemaphoreADC_DataReady_attributes = {
  .name = "SemaphoreADC_DataReady"
};
/* Definitions for SemaphoreUART_TxComplete */
osSemaphoreId_t SemaphoreUART_TxCompleteHandle;
const osSemaphoreAttr_t SemaphoreUART_TxComplete_attributes = {
  .name = "SemaphoreUART_TxComplete"
};
/* Definitions for SemaphoreFlagsPID */
osSemaphoreId_t SemaphoreFlagsPIDHandle;
const osSemaphoreAttr_t SemaphoreFlagsPID_attributes = {
  .name = "SemaphoreFlagsPID"
};
/* Definitions for SemaphoreFlagsIMU */
osSemaphoreId_t SemaphoreFlagsIMUHandle;
const osSemaphoreAttr_t SemaphoreFlagsIMU_attributes = {
  .name = "SemaphoreFlagsIMU"
};
/* Definitions for SemaphoreFlagsMOT */
osSemaphoreId_t SemaphoreFlagsMOTHandle;
const osSemaphoreAttr_t SemaphoreFlagsMOT_attributes = {
  .name = "SemaphoreFlagsMOT"
};
/* Definitions for SemaphoreFlagsOTH */
osSemaphoreId_t SemaphoreFlagsOTHHandle;
const osSemaphoreAttr_t SemaphoreFlagsOTH_attributes = {
  .name = "SemaphoreFlagsOTH"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float calculateAccAngle(float accY, float accZ, float ay_off, float az_off);
float complementaryFilter(float gyroX, float accAngle, float prevAngle, float gx_off);
float map(float x, float in_min, float in_max, float out_min, float out_max);
void Parser_PID_PrintCallback(uint8_t flag, float value);
void Parser_IMU_PrintCallback(uint8_t flag, float value);
void Parser_MOT_PrintCallback(uint16_t flag, uint16_t value);
void Parser_OTH_PrintCallback(uint8_t flag, float value);
Parser_StatusTypeDef Parser_ReceiveLineCallback(char* message);
void sendMessageToUART(char* msg);
/* USER CODE END FunctionPrototypes */

void StartUARTReadTask(void *argument);
void StartIMU_Task(void *argument);
void StartPIDTask(void *argument);
void StartStepperMotorsTask(void *argument);
void StartOtherTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	Parser_Register_PIDPrintCallback(Parser_PID_PrintCallback);
	Parser_Register_IMUPrintCallback(Parser_IMU_PrintCallback);
	Parser_Register_MotorsPrintCallback(Parser_MOT_PrintCallback);
	Parser_Register_OtherPrintCallback(Parser_OTH_PrintCallback);
	Parser_Register_ReceiveLineCallback(Parser_ReceiveLineCallback);
  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexI2C */
  MutexI2CHandle = osMutexNew(&MutexI2C_attributes);

  /* creation of MutexRingBufferRx */
  MutexRingBufferRxHandle = osMutexNew(&MutexRingBufferRx_attributes);

  /* creation of MutexRingBufferTx */
  MutexRingBufferTxHandle = osMutexNew(&MutexRingBufferTx_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreLSM6_DataReady */
  SemaphoreLSM6_DataReadyHandle = osSemaphoreNew(1, 1, &SemaphoreLSM6_DataReady_attributes);

  /* creation of SemaphoreADC_DataReady */
  SemaphoreADC_DataReadyHandle = osSemaphoreNew(1, 1, &SemaphoreADC_DataReady_attributes);

  /* creation of SemaphoreUART_TxComplete */
  SemaphoreUART_TxCompleteHandle = osSemaphoreNew(1, 1, &SemaphoreUART_TxComplete_attributes);

  /* creation of SemaphoreFlagsPID */
  SemaphoreFlagsPIDHandle = osSemaphoreNew(1, 1, &SemaphoreFlagsPID_attributes);

  /* creation of SemaphoreFlagsIMU */
  SemaphoreFlagsIMUHandle = osSemaphoreNew(1, 1, &SemaphoreFlagsIMU_attributes);

  /* creation of SemaphoreFlagsMOT */
  SemaphoreFlagsMOTHandle = osSemaphoreNew(1, 1, &SemaphoreFlagsMOT_attributes);

  /* creation of SemaphoreFlagsOTH */
  SemaphoreFlagsOTHHandle = osSemaphoreNew(1, 1, &SemaphoreFlagsOTH_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  SemRxLineAvailableHandle = osSemaphoreNew(31, 0, &SemRxLineAvailable_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueInputPID */
  QueueInputPIDHandle = osMessageQueueNew (8, sizeof(float), &QueueInputPID_attributes);

  /* creation of QueueOutputPID */
  QueueOutputPIDHandle = osMessageQueueNew (8, sizeof(int16_t), &QueueOutputPID_attributes);

  /* creation of QueueUartPID */
  QueueUartPIDHandle = osMessageQueueNew (1, sizeof(float), &QueueUartPID_attributes);

  /* creation of QueueUartIMU */
  QueueUartIMUHandle = osMessageQueueNew (1, sizeof(float), &QueueUartIMU_attributes);

  /* creation of QueueUartMOT */
  QueueUartMOTHandle = osMessageQueueNew (1, sizeof(uint16_t), &QueueUartMOT_attributes);

  /* creation of QueueUartOTH */
  QueueUartOTHHandle = osMessageQueueNew (1, sizeof(float), &QueueUartOTH_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UARTReadTask */
  UARTReadTaskHandle = osThreadNew(StartUARTReadTask, NULL, &UARTReadTask_attributes);

  /* creation of IMU_Task */
  IMU_TaskHandle = osThreadNew(StartIMU_Task, NULL, &IMU_Task_attributes);

  /* creation of PIDTask */
  PIDTaskHandle = osThreadNew(StartPIDTask, NULL, &PIDTask_attributes);

  /* creation of StepperMotorsTa */
  StepperMotorsTaHandle = osThreadNew(StartStepperMotorsTask, NULL, &StepperMotorsTa_attributes);

  /* creation of OtherTask */
  OtherTaskHandle = osThreadNew(StartOtherTask, NULL, &OtherTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUARTReadTask */
/**
  * @brief  Function implementing the UARTReadTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUARTReadTask */
void StartUARTReadTask(void *argument)
{
  /* USER CODE BEGIN StartUARTReadTask */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Tmp_Rx_Buffer, TMP_DMAUART_BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
	  if (osSemaphoreAcquire(SemRxLineAvailableHandle, 0) == osOK)
	  {
		  osMutexAcquire(MutexRingBufferRxHandle, osWaitForever);
	  	  Parser_ParseLine();
		  osMutexRelease(MutexRingBufferRxHandle);
	  }

	  else if (osMutexAcquire(MutexRingBufferTxHandle, 0) == osOK)
	  {
		  if(isReadable(&Tx_RingBuffer))
		  {
			  if(osSemaphoreAcquire(SemaphoreUART_TxCompleteHandle, 0) == osOK)
			  {
				  uint8_t i = 0;
				  while((i < TMP_DMAUART_BUFFER_SIZE) && (RB_Read(&Tx_RingBuffer, &Tmp_Tx_Buffer[i]) == RB_OK))
				  {
					  if(Tmp_Tx_Buffer[i] == '\n')
					  {
						  i++;
						  Tmp_Tx_Buffer[i] = '\0';
						  break;
					  }
					  i++;
				  }
				  HAL_UART_Transmit_DMA(&huart2, Tmp_Tx_Buffer, i);
				  osMutexRelease(MutexRingBufferTxHandle);
			  }
			  else
			  {
				  osMutexRelease(MutexRingBufferTxHandle);
				  osDelay(1);
			  }
		  }
		  else
		  {
			  osMutexRelease(MutexRingBufferTxHandle);
			  osDelay(1);
		  }
	  }

	  else osDelay(1);
  }
  /* USER CODE END StartUARTReadTask */
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
	float accAngle, outputAngle;
	float prevAngle = 0.0;
	float gyroX_Offset = IMU_GYRX_DEFAULT_OFFSET;
	float accY_Offset = IMU_ACCY_DEFAULT_OFFSET;
	float accZ_Offset = IMU_ACCZ_DEFAULT_OFFSET;
	uint16_t flags;
	char cmd[32];
	uint8_t autocalibrationStatus = 0;
	float tmpGyroX = 0;
	float tmpAccY = 0;
	float tmpAccZ = 0;
	uint8_t autocalibrationCounter = 0;

	osDelay(150*KERNEL_TICKS_FOR_MS);

	osMutexAcquire(MutexI2CHandle, osWaitForever);
	if (LSM6_InitEx(&hi2c1, &LSM6DS33, device_autoDetect, sa0_autoDetect) == false)
	{
		sendMessageToUART("Błąd połączenia z akcelerometrem!\r\n");
	}
	else
	{
		enableDefault(&LSM6DS33);
	}
	osMutexRelease(MutexI2CHandle);
  /* Infinite loop */
  for(;;)
  {
	if(osOK == osSemaphoreAcquire(SemaphoreLSM6_DataReadyHandle, 20*KERNEL_TICKS_FOR_MS))
	{
		osMutexAcquire(MutexI2CHandle, osWaitForever);
		if(HAL_OK == LSM6_Read(&LSM6DS33))
		{
			osMutexRelease(MutexI2CHandle);
			accAngle = calculateAccAngle(LSM6DS33.accelerometer.y, LSM6DS33.accelerometer.z, accY_Offset, accZ_Offset);
			outputAngle = complementaryFilter(LSM6DS33.gyroscope.x, accAngle, prevAngle, gyroX_Offset);
			osMessageQueuePut(QueueInputPIDHandle, &outputAngle, 0, 0);
//			if(ABS(prevAngle - outputAngle) > 0.09)
//			{
				sprintf(cmd, "ANG=%.1f\r\n", outputAngle);
				//sprintf(cmd, "%.1f\r\n", outputAngle);
				sendMessageToUART(cmd);
//			}
			prevAngle = outputAngle;

			if(autocalibrationStatus)
			{
				tmpGyroX += (LSM6DS33.gyroscope.x*IMU_GYRO_SENSITIVITY);
				tmpAccY  += (LSM6DS33.accelerometer.y*IMU_ACC_SENSITIVITY);
				tmpAccZ  += (LSM6DS33.accelerometer.z*IMU_ACC_SENSITIVITY);
				autocalibrationCounter++;
				if(autocalibrationCounter == IMU_AUTOCALIB_ITERATONS)
				{
					gyroX_Offset = -(tmpGyroX / IMU_AUTOCALIB_ITERATONS);
					accY_Offset  = -(tmpAccY  / IMU_AUTOCALIB_ITERATONS);
					accZ_Offset  = -(tmpAccZ  / IMU_AUTOCALIB_ITERATONS) + 1;
					autocalibrationCounter = 0;
					autocalibrationStatus = 0;
					tmpGyroX = 0;
					tmpAccY = 0;
					tmpAccZ = 0;
					sprintf(cmd, "CB=1\n");
					sendMessageToUART(cmd);
				}
			}
		}
		else
		{
			osMutexRelease(MutexI2CHandle);
			sendMessageToUART("Błąd odczytu akcelerometru!\r\n");
		}
	}
	if(osThreadFlagsWait(IMU_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < IMU_FLAGS)
	{
		float tmp;
		uint8_t isSetFlag;

		flags = osThreadFlagsGet();

		if((isSetFlag = CHECK_BIT(flags, PARSER_IMU_SET_FLAG)))
		{
			osMessageQueueGet(QueueUartIMUHandle, &tmp, 0, 0);
			osThreadFlagsClear(PARSER_IMU_SET_FLAG);
			CLEAR_BIT(flags, PARSER_IMU_SET_FLAG);
		}
		switch(flags)
		{
		case PARSER_IMU_GX_OFFSET_FLAG:
			if (isSetFlag) gyroX_Offset = tmp;
			else { sprintf(cmd, "GX=%.3f\r\n", gyroX_Offset); sendMessageToUART(cmd); }
			break;
		case PARSER_IMU_AY_OFFSET_FLAG:
			if (isSetFlag) accY_Offset = tmp;
			else { sprintf(cmd, "AY=%.3f\r\n", accY_Offset); sendMessageToUART(cmd); }
			break;
		case PARSER_IMU_AZ_OFFSET_FLAG:
			if (isSetFlag) accZ_Offset = tmp;
			else { sprintf(cmd, "AZ=%.3f\r\n", accZ_Offset); sendMessageToUART(cmd); }
			break;
		case PARSER_IMU_CALIBRATION_FLAG:
			if (isSetFlag && ((uint8_t) tmp == 1))
			{
				if(DRV8834_getRobotState() == STOPPED) autocalibrationStatus = 1;
				else
				{
					sprintf(cmd, "CB=0\r\n");
					sendMessageToUART(cmd);
					sprintf(cmd, "Wylacz silniki!\r\n");
					sendMessageToUART(cmd);
				}
			}
			else if(isSetFlag && ((uint8_t) tmp == 0)) autocalibrationStatus = 0;
			break;
		case PARSER_IMU_ANGLE_FLAG:
			if(!isSetFlag) { sprintf(cmd, "ANG=%.2f\r\n", outputAngle); sendMessageToUART(cmd); }
			break;
		default: break;
		}
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsIMUHandle);
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
	float input, output;
	float setpoint = PID_DEFAULT_SETPOINT;
	float self_balance_adjusting = 0;
	float setpoint_adjusted = PID_DEFAULT_SETPOINT;
	int16_t speed;
	uint16_t flags;
	RoborState_t state;
	char cmd[16];

	PID_Init(&PID, &input, &output, &setpoint_adjusted, PID_DEFAULT_KP, PID_DEFAULT_KI,
			PID_DEFAULT_KD, P_ON_ERROR, DIRECT);
	PID_SetMode(&PID, AUTOMATIC);
	PID_SetSampleTime(&PID, (uint32_t) ((1.0/(float)PID_SAMPLE_TIME) * 1000.0));
	PID_SetOutputLimits(&PID, -DRV8834_getMaxSpeed(), DRV8834_getMaxSpeed());
  /* Infinite loop */
  for(;;)
  {
	  if(osMessageQueueGet(QueueInputPIDHandle, &input, NULL, 20*KERNEL_TICKS_FOR_MS) == osOK)
	  {
		state = DRV8834_getRobotState();
		if(state != STOPPED)
		{
			if((input < ROBOT_MAX_ANGLE) && (input > -ROBOT_MAX_ANGLE))
			{
				switch(state)
				{
				case FORWARD:
					if(setpoint < (PID_DEFAULT_SETPOINT + 3)) setpoint += 0.1;
					if(output < -0.25*DRV8834_getMaxSpeed()) setpoint += 0.01;
					break;
				case BACKWARD:
					if(setpoint > (PID_DEFAULT_SETPOINT - 3)) setpoint -= 0.1;
					if(output > 0.25*DRV8834_getMaxSpeed()) setpoint -= 0.01;
					break;
				default:
					if(setpoint > 0.5) setpoint -= 0.1;
					else if(setpoint < -0.5) setpoint += 0.1;
					else setpoint = 0;
					break;
				}
				if(setpoint == 0)
				{
					if(output < 0) self_balance_adjusting += 0.001;
					else if(output > 0) self_balance_adjusting -= 0.001;
				}
				setpoint_adjusted = setpoint + self_balance_adjusting;
				PID_Compute(&PID);
			}
			else
			{
				output = 0;
			}

			if((output < ROBOT_ZERO_HYSTERESIS) && (output > -ROBOT_ZERO_HYSTERESIS))
			{
				output = 0;
			}
		}
		else
		{
			output = 0;
			setpoint = setpoint_adjusted = PID_DEFAULT_SETPOINT;
		}
		speed = (int16_t) output;
		osMessageQueuePut(QueueOutputPIDHandle, &speed, 0, 0);

		pidMonitorInput = input;
		pidMonitorOutput = output;
		pidMonitorSetpoint = setpoint_adjusted;
	  }

	  if(osThreadFlagsWait(PID_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < PID_FLAGS)
	  {
		float tmp;
		uint8_t isSetFlag;

		flags = osThreadFlagsGet();

		if((isSetFlag = CHECK_BIT(flags, PARSER_PID_SET_FLAG)))
		{
			osMessageQueueGet(QueueUartPIDHandle, &tmp, 0, 0);
			osThreadFlagsClear(PARSER_PID_SET_FLAG);
			CLEAR_BIT(flags, PARSER_PID_SET_FLAG);
		}
		switch(flags)
		{
		case PARSER_PID_KP_FLAG:
			if (isSetFlag)
				PID.dispKp = tmp;
			else
			{
				sprintf(cmd, "KP=%.3f\r\n", PID.dispKp);
				sendMessageToUART(cmd);
			}
			break;
		case PARSER_PID_KI_FLAG:
			if (isSetFlag) PID.dispKi = tmp;
			else { sprintf(cmd, "KI=%.3f\r\n", PID.dispKi); sendMessageToUART(cmd); }
			break;
		case PARSER_PID_KD_FLAG:
			if (isSetFlag) PID.dispKd = tmp;
			else { sprintf(cmd, "KD=%.3f\r\n", PID.dispKd); sendMessageToUART(cmd); }
			break;
		default: break;
		}
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsPIDHandle);
	  }
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
	int16_t speed = 0;
	int16_t speedLeft = 0;
	int16_t speedRight = 0;
	uint16_t flags;
	char cmd[16];
	RoborState_t state;
	uint8_t statePrintDelay = 0;

	DRV8834_Init(&leftMotor, &htim2, TIM_CHANNEL_3);
	DRV8834_InitPins(&leftMotor, leftMotor_DIR_Pin, leftMotor_DIR_GPIO_Port,
			leftMotor_M0_Pin, leftMotor_M0_GPIO_Port, leftMotor_M1_Pin, leftMotor_M1_GPIO_Port);
	DRV8834_SetMicrostep(&leftMotor, MOT_DEFAULT_MICROSTEP);

	DRV8834_Init(&rightMotor, &htim4, TIM_CHANNEL_1);
	DRV8834_InitPins(&rightMotor, rightMotor_DIR_Pin, rightMotor_DIR_GPIO_Port,
			rightMotor_M0_Pin, rightMotor_M0_GPIO_Port, rightMotor_M1_Pin, rightMotor_M1_GPIO_Port);
	DRV8834_SetMicrostep(&rightMotor, MOT_DEFAULT_MICROSTEP);

	DRV8834_StartMotor(&leftMotor, 0);
	DRV8834_StartMotor(&rightMotor, 0);
  /* Infinite loop */
  for(;;)
  {
	if(osOK == osMessageQueueGet(QueueOutputPIDHandle, &speed, NULL, 20*KERNEL_TICKS_FOR_MS))
	{
		state = DRV8834_getRobotState();
		switch(state)
		{
		case LEFT:  speedLeft = speed + MOT_TURNING_SPEED; speedRight = speed - MOT_TURNING_SPEED; break;
		case RIGHT:	speedLeft = speed - MOT_TURNING_SPEED; speedRight = speed + MOT_TURNING_SPEED; break;
		default: speedLeft = speedRight = speed; break;
		}
		if(state != STOPPED)
		{
			DRV8834_SetSpeed(&leftMotor, -speedLeft);
			DRV8834_SetSpeed(&rightMotor, speedRight);
		}

		sprintf(cmd, "SPD=%d\r\n", -(int)speed);
		sendMessageToUART(cmd);

		statePrintDelay++;
		if(statePrintDelay >= 25)
		{
			sprintf(cmd, "ST=%d\r\n", (int) state);
			sendMessageToUART(cmd);
			statePrintDelay = 0;
		}

	}

	  if(osThreadFlagsWait(MOT_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < MOT_FLAGS)
	  {
		uint16_t tmp;
		uint8_t isSetFlag;

		flags = osThreadFlagsGet();

		if((isSetFlag = CHECK_BIT(flags, PARSER_MOT_SET_FLAG)))
		{
			osMessageQueueGet(QueueUartMOTHandle, &tmp, 0, 0);
			osThreadFlagsClear(PARSER_MOT_SET_FLAG);
			CLEAR_BIT(flags, PARSER_MOT_SET_FLAG);
		}
		switch(flags)
		{
		case PARSER_MOT_FRWD_FLAG:
			if (isSetFlag) DRV8834_setRobotState((tmp > 0.5 ? FORWARD : BALANCING));
			break;
		case PARSER_MOT_BKWD_FLAG:
			if (isSetFlag) DRV8834_setRobotState((tmp > 0.5 ? BACKWARD : BALANCING));
			break;
		case PARSER_MOT_LEFT_FLAG:
			if (isSetFlag) DRV8834_setRobotState((tmp > 0.5 ? LEFT : BALANCING));
			break;
		case PARSER_MOT_RGHT_FLAG:
			if (isSetFlag) DRV8834_setRobotState((tmp > 0.5 ? RIGHT : BALANCING));
			break;
		case PARSER_MOT_MICROSTEP_FLAG:
			if (isSetFlag)
			{
				DRV8834_SetMicrostep(&leftMotor, (uint8_t) tmp);
				DRV8834_SetMicrostep(&rightMotor, (uint8_t) tmp);
			}
			else
			{ 	if(leftMotor.microstep == rightMotor.microstep)
					sprintf(cmd, "MST=%d\r\n", leftMotor.microstep);
				else
					sprintf(cmd, "Rozne mkroki!\r\n");
				sendMessageToUART(cmd);
			}
			break;
		case PARSER_MOT_MAXFREQ_FLAG:
			if (isSetFlag) DRV8834_setMaxFreq((uint16_t) tmp);
			else { sprintf(cmd, "MF=%d\r\n", (int) DRV8834_getMaxFreq()); sendMessageToUART(cmd); }
			break;
		case PARSER_MOT_MAXSPED_FLAG:
			if (isSetFlag) DRV8834_setMaxSpeed((uint16_t) tmp);
			else { sprintf(cmd, "MSP=%d\r\n", (int) DRV8834_getMaxSpeed()); sendMessageToUART(cmd); }
			break;
		case PARSER_MOT_BLOCK_FLAG:
			if (isSetFlag)
			{
				if(DRV8834_getRobotState() != STOPPED && ((uint8_t) tmp == 1))
					{ DRV8834_StopMotor(&leftMotor); DRV8834_StopMotor(&rightMotor); }
				else if(((uint8_t) tmp == 0))
					{ DRV8834_StartMotor(&leftMotor, speedLeft); DRV8834_StartMotor(&rightMotor, speedRight); }
			}
			break;
		case PARSER_MOT_SPEED_FLAG:
			if (!isSetFlag) { sprintf(cmd, "SPD=%d\r\n", (int)speed); sendMessageToUART(cmd); }
			break;
		case PARSER_MOT_STATE_FLAG:
			if (!isSetFlag) { sprintf(cmd, "ST=%d\r\n", (int)DRV8834_getRobotState()); sendMessageToUART(cmd); }
			break;
		default: break;
		}
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsMOTHandle);
	  }
  }
  /* USER CODE END StartStepperMotorsTask */
}

/* USER CODE BEGIN Header_StartOtherTask */
/**
* @brief Function implementing the OtherTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOtherTask */
void StartOtherTask(void *argument)
{
  /* USER CODE BEGIN StartOtherTask */
	uint16_t batteryVoltageReadings[OTH_BAT_NUM_OF_READINGS];
	float batteryVoltageMin = OTH_BAT_DEFAULT_MIN_V;
	float batteryVoltageMax = ((OTH_BAT_DEFAULT_MAX_V > OTH_ADC_VDDA_VALUE) ? OTH_ADC_VDDA_VALUE : OTH_BAT_DEFAULT_MAX_V);
	uint32_t batteryVoltageMean = 0;
	uint8_t batteryLevel = 0;
	uint16_t flags;
	char cmd[16];

	osSemaphoreAcquire(SemaphoreADC_DataReadyHandle, osWaitForever);
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)batteryVoltageReadings, OTH_BAT_NUM_OF_READINGS);
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(SemaphoreADC_DataReadyHandle, 200*KERNEL_TICKS_FOR_MS) == osOK)
	  {
		  float tmp;
		  batteryVoltageMean = 0;
		  for(uint8_t i=0; i<OTH_BAT_NUM_OF_READINGS; i++)
		  {
			  batteryVoltageMean += batteryVoltageReadings[i];
		  }
		  batteryVoltageMean /= OTH_BAT_NUM_OF_READINGS;

		  tmp = map(batteryVoltageMean, 0, OTH_ADC_MAX_VALUE, 0, OTH_ADC_VDDA_VALUE);
		  if(tmp < batteryVoltageMin)
			  tmp = batteryVoltageMin;

		  batteryLevel = (uint8_t) map(tmp, batteryVoltageMin, batteryVoltageMax, 0, 100);

		  sprintf(cmd, "BL=%d\r\n", (int) batteryLevel);
		  sendMessageToUART(cmd);

	  }

	  if(osThreadFlagsWait(PID_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < PID_FLAGS)
	  {
		float tmp;
		uint8_t isSetFlag;

		flags = osThreadFlagsGet();

		if((isSetFlag = CHECK_BIT(flags, PARSER_OTH_SET_FLAG)))
		{
			osMessageQueueGet(QueueUartOTHHandle, &tmp, 0, 0);
			osThreadFlagsClear(PARSER_OTH_SET_FLAG);
			CLEAR_BIT(flags, PARSER_OTH_SET_FLAG);
		}
		switch(flags)
		{
		case PARSER_OTH_BAT_LEVL_FLAG:
			if (!isSetFlag) { sprintf(cmd, "BL=%d\r\n", (int) batteryLevel); sendMessageToUART(cmd); }
			break;
		case PARSER_OTH_BAT_MINV_FLAG:
			if (isSetFlag) batteryVoltageMin = tmp;
			else { sprintf(cmd, "BMN=%.2f\r\n", batteryVoltageMin); sendMessageToUART(cmd); }
			break;
		case PARSER_OTH_BAT_MAXV_FLAG:
			if (isSetFlag) batteryVoltageMax = tmp;
			else {
				sprintf(cmd, "BMX=%.2f\r\n", ((batteryVoltageMax > OTH_ADC_VDDA_VALUE) ? OTH_ADC_VDDA_VALUE : batteryVoltageMax));
				sendMessageToUART(cmd); }
			break;
		default: break;
		}
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsOTHHandle);
	  }
  }
  /* USER CODE END StartOtherTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// ********************* GLOBAL INTERRUPT SECTION ********************* //

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == LSM6_Interrupt_Pin)
	{
		osSemaphoreRelease(SemaphoreLSM6_DataReadyHandle);
	}

	if(GPIO_Pin == LSM6_Interrupt2_Pin)
	{
		;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		osSemaphoreRelease(SemaphoreADC_DataReadyHandle);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2)
	{
		osMutexAcquire(MutexRingBufferRxHandle, osWaitForever);
		for(uint16_t i=0; i<Size; i++)
		{
			RB_Write(&Rx_RingBuffer, Tmp_Rx_Buffer[i]);

			if(Tmp_Rx_Buffer[i] == '\n')
			{
				RxLines++;
				osSemaphoreRelease(SemRxLineAvailableHandle);
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Tmp_Rx_Buffer, TMP_DMAUART_BUFFER_SIZE);
		osMutexRelease(MutexRingBufferRxHandle);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		osSemaphoreRelease(SemaphoreUART_TxCompleteHandle);
	}
}

// ******************* GLOBAL INTERRUPT SECTION END ******************* //

// ********************* LIBRARIES CALLBACK SECTION ********************* //

Parser_StatusTypeDef Parser_ReceiveLineCallback(char* message)
{
	  uint8_t i = 0;
	  uint8_t character;

	  while(RB_Read(&Rx_RingBuffer, &character) == RB_OK)
	  {
		  if(character == '\n')
		  {
			  *(message + i) = 0;
			  RxLines--;
			  return PARSER_OK;
		  }
		  *(message + i) = character;
		  i++;
	  }
	  return PARSER_ERROR;
}

void Parser_PID_PrintCallback(uint8_t flag, float value)
{
	if(CHECK_BIT(flag, PARSER_PID_SET_FLAG))
		osMessageQueuePut(QueueUartPIDHandle, &value, 0, osWaitForever);

	osSemaphoreAcquire(SemaphoreFlagsPIDHandle, osWaitForever);
	osThreadFlagsSet(PIDTaskHandle, flag);
}

void Parser_IMU_PrintCallback(uint8_t flag, float value)
{
	if(CHECK_BIT(flag, PARSER_IMU_SET_FLAG))
		osMessageQueuePut(QueueUartIMUHandle, &value, 0, osWaitForever);

	osSemaphoreAcquire(SemaphoreFlagsIMUHandle, osWaitForever);
	osThreadFlagsSet(IMU_TaskHandle, flag);
}

void Parser_MOT_PrintCallback(uint16_t flag, uint16_t value)
{
	if(CHECK_BIT(flag, PARSER_MOT_SET_FLAG))
		osMessageQueuePut(QueueUartMOTHandle, &value, 0, osWaitForever);

	osSemaphoreAcquire(SemaphoreFlagsMOTHandle, osWaitForever);
	osThreadFlagsSet(StepperMotorsTaHandle, flag);
}

void Parser_OTH_PrintCallback(uint8_t flag, float value)
{
	if(CHECK_BIT(flag, PARSER_OTH_SET_FLAG))
		osMessageQueuePut(QueueUartOTHHandle, &value, 0, osWaitForever);

	osSemaphoreAcquire(SemaphoreFlagsOTHHandle, osWaitForever);
	osThreadFlagsSet(OtherTaskHandle, flag);
}

// ******************* LIBRARIES CALLBACK SECTION END ******************* //

// ********************* OTHER FUNCTIONS SECTION ********************* //

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float complementaryFilter(float gyroX, float accAngle, float prevAngle, float gx_off)
{
	return (COMP_FILTER_GAIN * (prevAngle + ((gyroX*IMU_GYRO_SENSITIVITY + gx_off)/IMU_SAMPLE_TIME)))
			+ ((1 - COMP_FILTER_GAIN) * accAngle);
}

float calculateAccAngle(float accY, float accZ, float ay_off, float az_off)
{
	return (atan2((accY*IMU_ACC_SENSITIVITY + ay_off), (accZ*IMU_ACC_SENSITIVITY + az_off)) * 180.0/M_PI);
}

void sendMessageToUART(char* msg)
{
	osMutexAcquire(MutexRingBufferTxHandle, osWaitForever);
	uint8_t length = strlen(msg);
	for(uint8_t i = 0; i < length; i++)
		RB_Write(&Tx_RingBuffer, (uint8_t) *(msg+i));
	osMutexRelease(MutexRingBufferTxHandle);
}

// ******************* OTHER FUNCTIONS SECTION END ******************* //

/* USER CODE END Application */

