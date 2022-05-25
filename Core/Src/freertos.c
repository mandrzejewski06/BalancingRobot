/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Kod główny robota balansującego, które wykorzystuje
  * 					 system czasu rzeczywistego.
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

// Ustawienie 1 poniżej włącza zmienne globalne monitorujące
// Wejścia, wyjścia i nastawy regulatorów do wykorzystania w StmCubeMonitor
#define USE_CUBE_MONITOR 0

// Stałe inne
#define KERNEL_TICKS_FOR_MS 	1	// ile ticków FreeRTOS'a przypada na 1ms
#define TMP_DMAUART_BUFFER_SIZE 64	// Rozmiar tymczasowego buffera dla DMA

// Stałe opisujące ogólne ustawiena robota
#define ROBOT_ZERO_HYSTERESIS 	5		// Domyślna wartość zerowej histerezy prędkości
#define ROBOT_MAX_ANGLE			40		// Moduł max kąta wychylenia, po przekroczeniu którego wyłączane są silniki
#define ROBOT_COMPFILTER_GAIN	0.96	// Domyślna wartość wzmocnienia filtru komplementarnego
#define ROBOT_MOVING_SPEED		30		// Domyślna prędkość jazdy do przodu i do tyłu
#define ROBOT_TURNING_SPEED 	15		// Domyślna prędkość obrotu robota w lewo i prawo

// Stałe opisujące ustawienia akcelerometru i żyroskopu
#define IMU_SAMPLE_TIME 		52 			// Częstotliwość pomiaru IMU w [Hz]
#define IMU_GYRO_SENSITIVITY 	0.00875		// Mnożnik surowej wartości z żyroskopu do zakresu FS = +-245dps
#define IMU_ACC_SENSITIVITY		0.000061	// Mnożnik surowej wartości z akcelerometru do zakresu FS = +-2g
#define IMU_ACCY_DEFAULT_OFFSET	0.05		// Domyślna wartość offsetu dla osi X żyroskopu
#define IMU_ACCZ_DEFAULT_OFFSET	-0.02		// Domyślna wartość offsetu dla osi Y akcelerometru
#define IMU_GYRX_DEFAULT_OFFSET	-2.0		// Domyślna wartość offsetu dla osi Z akcelerometru
#define IMU_AUTOCALIB_ITERATONS	100			// Ilość próbek pobieranych do autokalibracji (wyciągnięta zostaje średnia z tylu pomiarów)

// Stałe opisujące regulatory PID
#define PID_SAMPLE_TIME 	 	IMU_SAMPLE_TIME	// Częstotliwość wykonywania obliczeń regulatorów
#define PID_DEFAULT_SETPOINT 	0.0				// Domyślny setpoint dla obu regulatorów
#define PID_ROBOT_MAX_LEAN	 	3				// Domyślny maks kąt wychylenia zadawany przez regulator prędkości
#define PID1_DEFAULT_KP 		0.05			// Domyślne wzmocnienie członu proporcjonalnego Kp regulatora prędkości (PID1)
#define PID1_DEFAULT_KI 		0.1				// Domyślne wzmocnienie członu całkującego Ki regulatora PID1
#define PID1_DEFAULT_KD 		0.0				// Domyślne wzmocnienie członu różniczkującego Kd regulatora PID1
#define PID2_DEFAULT_KP 		3.0				// Domyślne wzmocnienie członu proporcjonalnego Kp regulatora kąta wychylenia (PID2)
#define PID2_DEFAULT_KI 		24.0			// Domyślne wzmocnienie członu całkującego Ki regulatora PID2
#define PID2_DEFAULT_KD 		0.02			// Domyślne wzmocnienie członu różniczkującego Kd regulatora PID2

// Stałe opisujące silniki krokowe
// Większość stałych jest umieszczona w pliku DRV8834.h
#define MOT_DEFAULT_MICROSTEP 	4	// Domyślna wartość mikrokroku

// Stałe opisujące dodatkowe funkcjonalności
#define OTH_BAT_NUM_OF_READINGS 10		// Ilość próbek zbieranych przez ADC do pomiaru napięcia baterii
#define OTH_BAT_DEFAULT_MIN_V	2.09	// Wartość napięcia na ADC oznaczająca rozładowanie baterii (dla 3 Li-po 18650 w szeregu jest to ok. 9V)
#define OTH_BAT_DEFAULT_MAX_V	2.93	// Wartość maks napięcia na ADC przy pełnym naładowaniu baterii (12,6V)
#define OTH_ADC_MAX_VALUE		4096	// Maksymalna wartość adc przy wybranej rozdzielczości
#define OTH_ADC_VDDA_VALUE		3.3		// Wartość napięcia zasilającego ADC

// Stałe pomocnicze opisujące wykorzystywane flagi dla każdego z tasków
#define PID_FLAGS 		0xFF
#define IMU_FLAGS 		0xFF
#define MOT_FLAGS 		0xFFF
#define OTH_FLAGS 		0xFFF
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CHECK_BIT(FLAG, BIT) ((BIT) == ((FLAG)&(BIT)))	// Makro sprawdzające czy podany bit we fladze jest ustawiony
#define ABS(value) (value < 0 ? -value : value)			// Makro zwracające wartość bezwzględną podanej zmiennej
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float COMP_FILTER_GAIN = ROBOT_COMPFILTER_GAIN;	// Zmienna przechowująca wartość wzmocnienia filtru komplementarnego

// Zmienne dla CubeMonitor
#if(USE_CUBE_MONITOR == 1)
uint8_t RxLines = 0;
float pidMonitorInput;
float pidMonitorOutput;
float pidMonitorSetpoint;
float pidMonitorSetpoint2;
#endif

// Tymczasowe buffery dla DMA do UART Rx i Tx
uint8_t Tmp_Rx_Buffer[TMP_DMAUART_BUFFER_SIZE];
uint8_t Tmp_Tx_Buffer[TMP_DMAUART_BUFFER_SIZE];
// Ring buffery dla odczytów z UARTA i bajtów do wysłania przez UART
RingBuffer_t Rx_RingBuffer;
RingBuffer_t Tx_RingBuffer;

// Semafor zliczający dostępne linie do odczytu przez parser
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
float calculateAccAngle(float accY, float accZ, float ay_off, float az_off);	// Funkcja obliczająca kąt wychylenia na podstawie akcelerometru
float complementaryFilter(float gyroX, float accAngle, float prevAngle, float gx_off);	// Funkcja implementująca filtr komplementarny
float map(float x, float in_min, float in_max, float out_min, float out_max);	// Funkcja pomocnicza do skalowania zmiennej do podanego zakresu

// Callbacki do funkcji parsera
void Parser_PID_PrintCallback(uint8_t flag, float value);
void Parser_IMU_PrintCallback(uint8_t flag, float value);
void Parser_MOT_PrintCallback(uint16_t flag, uint16_t value);
void Parser_OTH_PrintCallback(uint8_t flag, float value);
Parser_StatusTypeDef Parser_ReceiveLineCallback(char* message);

void sendMessageToUART(char* msg);	// Funkcja, którą wykorzystują taski do wysyłania informacji przez UART
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
	// Rejestracja Callbacków dla biblioteki parsera
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
  // Ten semafor stworzony jest bez generowania z CubeMx, ponieważ domyślnie posiada 0 tokenów po starcie programu
  // i zwiększa ich ilość wraz z każdą dostepną linią do odczytu z Rx ring buffera
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
  * @brief  Task (zadanie), które obsługuje odczyt i zapis przez UART (wykorzystując DMA)
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUARTReadTask */
void StartUARTReadTask(void *argument)
{
  /* USER CODE BEGIN StartUARTReadTask */
	// Ustawienie nasłuchu przez DMA linii Rx UART'a. Przerwanie wystąpi po odczycie TMP_DMAUART_BUFFER_SIZE danych
	// lub po wykryciu przestoju (przerwy, ciszy) na linii Rx (linia Rx w stanie IDLE).
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Tmp_Rx_Buffer, TMP_DMAUART_BUFFER_SIZE);
  /* Infinite loop */
  for(;;)
  {
	  // Sprawdzenie czy jakieś linie są dostępne do odczytu z Rx RingBuffera
	  if (osSemaphoreAcquire(SemRxLineAvailableHandle, 0) == osOK)
	  {
		  // Mutex dla bezpieczeństwa w przypadku gdyby jakies dane były aktualnie zapisywane przez DMA do Rx RingBuffera
		  osMutexAcquire(MutexRingBufferRxHandle, osWaitForever);
	  	  Parser_ParseLine();	// Parsuj jedną linie
		  osMutexRelease(MutexRingBufferRxHandle);
	  }
	  // Sprawdzenie czy jakieś dane są aktualnie zapisywane do Tx RingBuffera
	  else if (osMutexAcquire(MutexRingBufferTxHandle, 0) == osOK)
	  {
		  // Sprawdzenie czy w ogóle jakieś dane znajdują się w Tx RingBufferze, które można wysłać (sprawdzenie czy RingBuffer nie jest pusty)
		  if(isReadable(&Tx_RingBuffer))
		  {
			  // Sprawdzenie czy poprzednia wysyłka jednej linii przez DMA się zakończyła
			  if(osSemaphoreAcquire(SemaphoreUART_TxCompleteHandle, 0) == osOK)
			  {
				  uint8_t i = 0;
				  // Pobierz jedną linię (komendę) do wysyłki przez DMA i UART
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
				  // Wyślij przez DMA i UART jedną komendę
				  HAL_UART_Transmit_DMA(&huart2, Tmp_Tx_Buffer, i);
				  // Zwolnij Mutex umożliwiając dalszy zapis komend do RingBuffera
				  osMutexRelease(MutexRingBufferTxHandle);
			  }
			  else
			  {
				  osMutexRelease(MutexRingBufferTxHandle); // Zwolnij Mutex umożliwiając dalszy zapis komend do RingBuffera
				  osDelay(1); // Umożliw wykonanie się taskowi o niższym priorytecie w najbliższej milisekundzie
			  }
		  }
		  else
		  {
			  osMutexRelease(MutexRingBufferTxHandle); // Zwolnij Mutex umożliwiając dalszy zapis komend do RingBuffera
			  osDelay(1);	// Umożliw wykonanie się taskowi o niższym priorytecie w najbliższej milisekundzie
		  }
	  }

	  else osDelay(1);	// Umożliw wykonanie się taskowi o niższym priorytecie w najbliższej milisekundzie
  }
  /* USER CODE END StartUARTReadTask */
}

/* USER CODE BEGIN Header_StartIMU_Task */
/**
* @brief Task, który obsługuje komunikację z modułem IMU oraz oblicza wartość kąta wychylenia wykorzystując
* 		 filtr komplementarny. Komunikacja z modułem po I2C wykonana blokująco (bez DMA czy przerwań), ponieważ
* 		 jest to pomiar krytyczny dla programu robota balansującego, bez którego nie można wykonać dalszych zadań.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU_Task */
void StartIMU_Task(void *argument)
{
  /* USER CODE BEGIN StartIMU_Task */
	LSM6_t LSM6DS33;	// Stworzenie obiektu IMU
	float accAngle, outputAngle;
	float prevAngle = 0.0;
	float gyroX_Offset = IMU_GYRX_DEFAULT_OFFSET;
	float accY_Offset = IMU_ACCY_DEFAULT_OFFSET;
	float accZ_Offset = IMU_ACCZ_DEFAULT_OFFSET;
	uint16_t flags;	// zmienna pomocnicza dla flag przesyłanych do zadania przez parser
	char cmd[16];	// Buffer pomocniczy do wysyłania wiadomości (komend) przez UART

	uint8_t autocalibrationStatus = 0;	// Status autokalibracji (1 - start kalibracji, 0 - ustawiane po wykonaniu autokalibracji)
	uint8_t autocalibrationCounter = 0;	// Zmienna zliczająca liczbe wykonanych iteracji autokalibracji
	// Zmienne pomocnicze do zliczania średniej z pomiarów do autokalibracji
	float tmpGyroX = 0;
	float tmpAccY = 0;
	float tmpAccZ = 0;

	// Po podłączeniu zasilania moduł IMU wymaga krótkiego czasu, dla własnej autoinicjalizacji. Nie można wtedy ustawiać w nim żadnych rejestrów
	osDelay(150*KERNEL_TICKS_FOR_MS);

	// Po odczekaniu zablokuj dostęp do I2C dla innych zadań
	osMutexAcquire(MutexI2CHandle, osWaitForever);
	// Sprawdź czy jest połączenie z IMU i zainicjalizuj obiekt
	if (LSM6_InitEx(&hi2c1, &LSM6DS33, device_autoDetect, sa0_autoDetect) == false)
	{
		// Jeśli nie ma połączenia wyślij komunikat przez UART
		sendMessageToUART("#Błąd połączenia z akcelerometrem!\r\n");
	}
	else
	{
		// Jeśli jest połączenie ustaw rejestry konfiguracyjne (opis rejestrów wewnątrz poniższej funkcji)
		enableDefault(&LSM6DS33);
	}
	// Odblokuj dostęp do I2C dla innych zadań
	osMutexRelease(MutexI2CHandle);
  /* Infinite loop */
  for(;;)
  {
	  // Sprawdź czy przyszło przerwanie od IMU oznaczające, że pomiar jest gotowy do odczytu
	if(osOK == osSemaphoreAcquire(SemaphoreLSM6_DataReadyHandle, 20*KERNEL_TICKS_FOR_MS))
	{
		// Zablokuj dostęp do I2C dla innych zadań
		osMutexAcquire(MutexI2CHandle, osWaitForever);
		// Odczytaj pomiar z żyrokopu i akcelerometru
		if(HAL_OK == LSM6_Read(&LSM6DS33))
		{
			// Jeśli odczyt pomiaru jest poprawny
			// Odblokuj I2C dla innych zadań
			osMutexRelease(MutexI2CHandle);
			// Oblicz wartość kąta na podstawie akcelerometru
			accAngle = calculateAccAngle(LSM6DS33.accelerometer.y, LSM6DS33.accelerometer.z, accY_Offset, accZ_Offset);
			// Oblicz wartość kąta wykorzystując filtr komplementarny (wewnątrz zaszyto też pomiar kąta na podstawie żyroskopu)
			outputAngle = complementaryFilter(LSM6DS33.gyroscope.x, accAngle, prevAngle, gyroX_Offset);
			// Wrzuć obliczony kąt wychylenia do kolejki, z której korzysta regulator PID
			osMessageQueuePut(QueueInputPIDHandle, &outputAngle, 0, 0);

			// Wyślij przez UART wartość wykonanego pomiaru kąta)
			sprintf(cmd, "ANG=%.1f\r\n", outputAngle);
			sendMessageToUART(cmd);
			// Zapisz do pomocniczej zmienej wartość kąta, do obliczeń w kolejnej iteracji (jest to potrzebne do całkowania wartości żyroskopu)
			prevAngle = outputAngle;

			// Sprawdzenie czy nie otrzymano komendy autokalibracji lub czy kalibracja jest w trakcie wykonywania
			// Przed wykonaniem autokalibracji należy ustawić robota w pozycji idealnego balansu
			// Autokalibracja wyskaluje offsety tak, aby wartość kąta w tej pozycji wynosiłą 0 stopni
			if(autocalibrationStatus)
			{
				// Dodawaj w każdej iteracji wartości dla wykorzystywanych osi
				tmpGyroX += (LSM6DS33.gyroscope.x*IMU_GYRO_SENSITIVITY);
				tmpAccY  += (LSM6DS33.accelerometer.y*IMU_ACC_SENSITIVITY);
				tmpAccZ  += (LSM6DS33.accelerometer.z*IMU_ACC_SENSITIVITY);
				autocalibrationCounter++;
				// Sprawdzenie czy wykonano zadaną liczbę iteracji (zebrano zadaną liczbę próbek)
				if(autocalibrationCounter == IMU_AUTOCALIB_ITERATONS)
				{
					// Wylicz średnią z wszystkich próbek i zaktualizuj offsety
					gyroX_Offset = -(tmpGyroX / IMU_AUTOCALIB_ITERATONS);
					accY_Offset  = -(tmpAccY  / IMU_AUTOCALIB_ITERATONS);
					accZ_Offset  = -(tmpAccZ  / IMU_AUTOCALIB_ITERATONS) + 1; // Dodano 1, bo jest to oś na którą działa siła grawitacji (1g)
					// Wyzerowanie wszystkich zmiennych w przypadku kolenej autokalibracji
					autocalibrationCounter = 0;
					autocalibrationStatus = 0;
					tmpGyroX = 0;
					tmpAccY = 0;
					tmpAccZ = 0;
					// Wyślij komendę, że autokalibracja się zakończyła
					sprintf(cmd, "CB=1\n");
					sendMessageToUART(cmd);
				}
			}
		}
		else
		{
			// Jeśli wystąpił błąd podczas odczytu z IMU to wyślij komunikat przez UART
			// i odblokuj I2C dla innych zadań
			osMutexRelease(MutexI2CHandle);
			sendMessageToUART("#Błąd odczytu akcelerometru!\r\n");
		}
	}

	// Sprawdzenie czy otrzymano jakieś flagi (zapytania lub rozkazy zapisu do zmiennej)
	if(osThreadFlagsWait(IMU_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < IMU_FLAGS)
	{
		float tmp;
		uint8_t isSetFlag;
		// Pobranie otrzymanych flag
		flags = osThreadFlagsGet();

		// Sprawdzenie czy otrzymano flagę zapisu do danej zmiennej
		if((isSetFlag = CHECK_BIT(flags, PARSER_IMU_SET_FLAG)))
		{
			// Jeśli tak to pobierz wartość z kolejki
			osMessageQueueGet(QueueUartIMUHandle, &tmp, 0, 0);
			// Wyczyść flagę zapisu globalnie i w lokalnej zmiennej
			osThreadFlagsClear(PARSER_IMU_SET_FLAG);
			CLEAR_BIT(flags, PARSER_IMU_SET_FLAG);
		}

		// Sprawdź której zmiennej dotyczy flaga
		// Jeśli jest to komenda zapisu to przypisz wartość z kolejki do danej zmiennej
		// Jeśli jest to komenda odczytu to wyślij przez UART wartość tej zmiennej
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
		case PARSER_IMU_COMP_FILTER_FLAG:
			if (isSetFlag) COMP_FILTER_GAIN = tmp;
			else { sprintf(cmd, "AZ=%.2f\r\n", COMP_FILTER_GAIN); sendMessageToUART(cmd); }
			break;
		case PARSER_IMU_CALIBRATION_FLAG:
			// Jeśli otrzymano komendę CB=1 zacznij autokalibrację
			if (isSetFlag && ((uint8_t) tmp == 1))
			{
				// Sprawdź jeszcze czy robot znajduje się w stanie STOPPED
				if(DRV8834_getRobotState() == STOPPED) autocalibrationStatus = 1;
				else
				{
					// Jeśli nie jest w stanie STOPPED zwróć błąd
					sendMessageToUART("CB=0\r\n");
					sendMessageToUART("#Wylacz silniki!\r\n");
				}
			}
			// Jeśli otrzymano komendę CB=0 przerwij autokalibrację i wyzeruj wykorzystywane zmienne
			else if(isSetFlag && ((uint8_t) tmp == 0)) autocalibrationStatus = autocalibrationCounter = tmpAccY = tmpAccZ = tmpGyroX = 0;
			break;
		case PARSER_IMU_ANGLE_FLAG: // Zmienna kąta wychylenia jest wysyłana cyklicznie, ale można ją odczytać też na żądanie (read only)
			if(!isSetFlag) { sprintf(cmd, "ANG=%.2f\r\n", outputAngle); sendMessageToUART(cmd); }
			break;
		default: break;
		}
		// Wyczyść flagę i odblokuj mutex umożliwiając ustawienie kolejnej flagi przez parser
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsIMUHandle);
	}
  }
  /* USER CODE END StartIMU_Task */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Task, który wykonuje obliczenia regulatorów PID.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void *argument)
{
  /* USER CODE BEGIN StartPIDTask */
	// Stworzenie dwóch obiektów: regulatora kąta wychylenia i regulatora prędkości
	PID_t PID1; // input - speed, output - angle_setpoint, setpoint - speed_setpoint
	PID_t PID2; // input - angle, output - speed, setpoint - angle_setpoint

	// Stworzenie zmiennych wykorzystywamych przez regulatory zgodnie z opisem powyżej
	float angle, speed;
	float angle_setpoint = PID_DEFAULT_SETPOINT;	// Przypisanie początkowej wartości nastawy
	float speed_setpoint = PID_DEFAULT_SETPOINT;	// Przypisanie początkowej wartości nastawy
	uint16_t flags;		// zmienna pomocnicza dla flag przesyłanych do zadania przez parser
	int16_t queueSpeed;	// zmienna pomocnicza potrzebna do wysyłania wartości int16_t (a nie float) do kolejki zadania silników krokowych
	RoborState_t state;	// zmienna pomocnicza do przechowywania aktualnego stanu robota
	char cmd[16];		// Buffer pomocniczy do wysyłania wiadomości (komend) przez UART
	uint8_t robotMaxLean = PID_ROBOT_MAX_LEAN;				// Maks kąt wychylenia (jest to zakres wyjściowy dla PID1)
	uint8_t robotMovingSpeed = ROBOT_MOVING_SPEED;			// Docelowa prędkość (speed_setpoint) przy poruszaniu się do przodu i do tyłu
	uint8_t robotZeroHysteresis = ROBOT_ZERO_HYSTERESIS;	// Zakres prędkości, w którym prędkość jest sprowadzana do 0 (zapobiega to drganiu robota)

	// Inicjalizacja utworzonych regulatorów
	// Najpierw regulator kąta wychylenia PID2
	PID_Init(&PID2, &angle, &speed, &angle_setpoint, PID2_DEFAULT_KP, PID2_DEFAULT_KI,
			PID2_DEFAULT_KD, P_ON_ERROR, DIRECT);	// Tryb direct, bo przy zwiększaniu się kąta, regulator musi kontrować prędkością w tym samym kierunku
	PID_SetMode(&PID2, AUTOMATIC);	// Tryb automatyczny odblokowujący możliwość wywoływana funkcji PID_Compute()
	PID_SetSampleTime(&PID2, (uint32_t) ((1.0/(float)PID_SAMPLE_TIME) * 1000.0));	// Przeliczenie częstotliwości obliczeń na okres w [ms]
	PID_SetOutputLimits(&PID2, -DRV8834_getMaxSpeed(), DRV8834_getMaxSpeed());		// Zakres wyjściowy zależy od ustawionej maks prędkości silników

	// Teraz regulator prędkości PID1
	PID_Init(&PID1, &speed, &angle_setpoint, &speed_setpoint, PID1_DEFAULT_KP, PID1_DEFAULT_KI,
			PID1_DEFAULT_KD, P_ON_ERROR, REVERSE); // Tryb reverse, bo przy zwiększaniu prędkości, regulator musi kontrować nastawą kąta w przeciwnym kierunku
	PID_SetMode(&PID1, AUTOMATIC);
	PID_SetSampleTime(&PID1, (uint32_t) ((1.0/(float)PID_SAMPLE_TIME) * 1000.0));
	PID_SetOutputLimits(&PID1, -robotMaxLean, robotMaxLean);	// Zakres wyjściowy zależy od ustawionej maks wartości wychylenia robota
  /* Infinite loop */
  for(;;)
  {
	  // Sprawdzenie czy można pobrać wartość kąta wychylenia z kolejki
	  if(osMessageQueueGet(QueueInputPIDHandle, &angle, NULL, 20*KERNEL_TICKS_FOR_MS) == osOK)
	  {
		// Pobierz aktualny stan robota
		state = DRV8834_getRobotState();
		// Sprawdź czy silniki nie są zastopowane
		if(state != STOPPED)
		{
			// Sprawdź czy kąt jest na tyle mały, aby dało się jeszcze balansować
			if((angle < ROBOT_MAX_ANGLE) && (angle > -ROBOT_MAX_ANGLE))
			{
				// Wyznacz nastawę prędkości zgodnie z aktualnym stanem robota
				switch(state)
				{
				case FORWARD:
					speed_setpoint = -robotMovingSpeed;
					break;
				case BACKWARD:
					speed_setpoint = robotMovingSpeed;
					break;
				case LEFT:
				case RIGHT:
				default:
					speed_setpoint = PID_DEFAULT_SETPOINT;
					break;
				}
				// Wykonaj obliczenia, najpierw regulatora prędkości aby zaktualizować nastawę kąta wychylenia
				PID_Compute(&PID1);
				// Teraz oblicz prędkość na podstawie odczytu kąta z kolejki i zaktualizowanej nastawy
				PID_Compute(&PID2);
			}
			else
			{	// Jeśli kąt jest na tyle duży, że nie da się balansować ustaw prękość na 0 (robot przewraca się)
				speed = 0;
			}

			// Jeśli obliczona prędkość jest bardzo niska to zredukuj ją do 0
			// Zapobiega to drganiom robota przy niskich prędkościach np. przy prędkościach do 0.1*maxSpeed
			if((speed < robotZeroHysteresis) && (speed > -robotZeroHysteresis))
			{
				speed = 0;
			}
		}
		else
		{	// Jeśli robot jest w stanie STOPPED to wyzeruj prędkość
			speed = 0;
		}
		// Rzutowanie zmiennoprzecinkowej prędkości na liczbę całkowitą
		queueSpeed = (int16_t) speed;

		// Wysłanie aktualnych nastaw regulatorów przez UART
		sprintf(cmd, "P1+SP=%d\r\n", -(int)speed_setpoint);
		sendMessageToUART(cmd);
		sprintf(cmd, "P2+SP=%.2f\r\n", angle_setpoint);
		sendMessageToUART(cmd);

		// Wrzuć do kolejki dla silników krokowych wartość całkowitoliczbową prędkości
		osMessageQueuePut(QueueOutputPIDHandle, &queueSpeed, 0, 0);

		// Zaktualizuj zmienne globalne jeśli używany jest CubeMonitor
#if(USE_CUBE_MONITOR == 1)
		pidMonitorInput = angle;
		pidMonitorOutput = speed;
		pidMonitorSetpoint = angle_setpoint;
		pidMonitorSetpoint2 = speed_setpoint;
#endif
	  }

	  // Sprawdzenie czy otrzymano jakieś flagi (zapytania lub rozkazy zapisu do zmiennej)
	  if(osThreadFlagsWait(PID_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < PID_FLAGS)
	  {
		float tmp;
		uint8_t isSetFlag;
		uint8_t isPID2Flag;
		// Pobranie otrzymanych flag
		flags = osThreadFlagsGet();

		// Sprawdzenie czy otrzymano flagę zapisu do danej zmiennej
		if((isSetFlag = CHECK_BIT(flags, PARSER_PID_SET_FLAG)))
		{
			// Jeśli tak to pobierz wartość z kolejki
			osMessageQueueGet(QueueUartPIDHandle, &tmp, 0, 0);
			// Wyczyść flagę zapisu globalnie i w lokalnej zmiennej
			osThreadFlagsClear(PARSER_PID_SET_FLAG);
			CLEAR_BIT(flags, PARSER_PID_SET_FLAG);
		}
		// Sprawdzenie do którego regulatora odnosi się flaga
		if((isPID2Flag = CHECK_BIT(flags, PARSER_PID_PID2_FLAG)))
		{
			osThreadFlagsClear(PARSER_PID_PID2_FLAG);
			CLEAR_BIT(flags, PARSER_PID_PID2_FLAG);
		}

		// Sprawdź której zmiennej dotyczy flaga
		// Jeśli jest to komenda zapisu to przypisz wartość z kolejki do danej zmiennej
		// Jeśli jest to komenda odczytu to wyślij przez UART wartość tej zmiennej
		switch(flags)
		{
		case PARSER_PID_KP_FLAG:
			if (isSetFlag)
				isPID2Flag ? setKp(&PID2, tmp) : setKp(&PID1, tmp);
			else
			{
				isPID2Flag ? sprintf(cmd, "P2+KP=%.3f\r\n", PID2.dispKp) : sprintf(cmd, "P1+KP=%.3f\r\n", PID1.dispKp);
				sendMessageToUART(cmd);
			}
			break;
		case PARSER_PID_KI_FLAG:
			if (isSetFlag)
				isPID2Flag ? setKi(&PID2, tmp) : setKi(&PID1, tmp);
			else
			{
				isPID2Flag ? sprintf(cmd, "P2+KI=%.3f\r\n", PID2.dispKi) : sprintf(cmd, "P1+KI=%.3f\r\n", PID1.dispKi);
				sendMessageToUART(cmd);
			}
			break;
		case PARSER_PID_KD_FLAG:
			if (isSetFlag)
				isPID2Flag ? setKd(&PID2, tmp) : setKd(&PID1, tmp);
			else
			{
				isPID2Flag ? sprintf(cmd, "P2+KD=%.3f\r\n", PID2.dispKd) : sprintf(cmd, "P1+KD=%.3f\r\n", PID1.dispKd);
				sendMessageToUART(cmd);
			}
			break;
		case PARSER_PID_MAX_LEAN_FLAG:
			if (isSetFlag) robotMaxLean = tmp;
			else { sprintf(cmd, "#RML=%d\r\n", robotMaxLean); sendMessageToUART(cmd); }
			break;
		case PARSER_PID_MOV_SPEED_FLAG:
			if (isSetFlag) robotMovingSpeed = tmp;
			else { sprintf(cmd, "#RMS=%d\r\n", robotMovingSpeed); sendMessageToUART(cmd); }
			break;
		case PARSER_PID_ZERO_HYST_FLAG:
			if (isSetFlag) robotZeroHysteresis = tmp;
			else { sprintf(cmd, "#ZH=%d\r\n", robotZeroHysteresis); sendMessageToUART(cmd); }
			break;
		default: break;
		}
		// Wyczyść flagę i odblokuj mutex umożliwiając ustawienie kolejnej flagi przez parser
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsPIDHandle);
	  }
  }
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartStepperMotorsTask */
/**
* @brief Task, który wysterowuje silniki krokowe (pośrednio - przez moduły DRV8834)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStepperMotorsTask */
void StartStepperMotorsTask(void *argument)
{
  /* USER CODE BEGIN StartStepperMotorsTask */
	// Stwórz obiekty dwóch silników krokowych (modułów DRV8834)
	StepMotor_t rightMotor;
	StepMotor_t leftMotor;

	// Ustaw początkowe prędkości na 0
	int16_t speed = 0;
	int16_t speedLeft = 0;
	int16_t speedRight = 0;

	uint16_t flags; // Zmienna pomocnicza dla flag przesyłanych do zadania przez parser
	char cmd[16];	// Buffer pomocniczy do wysyłania wiadomości (komend) przez UART
	RoborState_t state;	// zmienna pomocnicza do przechowywania aktualnego stanu robota
	uint8_t statePrintDelay = 0;	// zmienna pomocnicza do rzadszego wysyłania przez UART stanu robota
	uint8_t robotTurningSpeed = ROBOT_TURNING_SPEED;	// maks prędkośc obrotu robota w lewo i prawo

	// Inicjalizacji silników krokowych
	// Najpierw lewy silnik
	DRV8834_Init(&leftMotor, &htim2, TIM_CHANNEL_3);	// Wstępna inicjalizacja i przpisanie timera PWM
	DRV8834_InitPins(&leftMotor, leftMotor_DIR_Pin, leftMotor_DIR_GPIO_Port,
			leftMotor_M0_Pin, leftMotor_M0_GPIO_Port, leftMotor_M1_Pin, leftMotor_M1_GPIO_Port);	// Przypisanie pinów GPIO
	DRV8834_SetMicrostep(&leftMotor, MOT_DEFAULT_MICROSTEP);	// Ustawienie domyślnego mikrokroku
	// Następnie prawy silnik
	DRV8834_Init(&rightMotor, &htim4, TIM_CHANNEL_1);	// Wstępna inicjalizacja i przpisanie timera PWM
	DRV8834_InitPins(&rightMotor, rightMotor_DIR_Pin, rightMotor_DIR_GPIO_Port,
			rightMotor_M0_Pin, rightMotor_M0_GPIO_Port, rightMotor_M1_Pin, rightMotor_M1_GPIO_Port);	// Przypisanie pinów GPIO
	DRV8834_SetMicrostep(&rightMotor, MOT_DEFAULT_MICROSTEP);	// Ustawienie domyślnego mikrokroku
	// Wystartowanie silników (wprowadzenie w tryb BALANCING) z zerową prędkością
	DRV8834_StartMotor(&leftMotor, 0);
	DRV8834_StartMotor(&rightMotor, 0);
  /* Infinite loop */
  for(;;)
  {
	// Sprawdzenie czy w kolejce znajduje się nową wartość prędkości
	if(osOK == osMessageQueueGet(QueueOutputPIDHandle, &speed, NULL, 20*KERNEL_TICKS_FOR_MS))
	{
		// Pobranie aktualnego stanu robota
		state = DRV8834_getRobotState();
		// Przypisanie prędkości do silników zgodnie z aktualnym trybem robota
		switch(state)
		{
		case LEFT:
			speedLeft = speed + robotTurningSpeed;
			speedRight = speed - robotTurningSpeed;
			break;
		case RIGHT:
			speedLeft = speed - robotTurningSpeed;
			speedRight = speed + robotTurningSpeed;
			break;
		case FORWARD:
		case BACKWARD:
		default:
			speedLeft = speed;
			speedRight = speed;
			break;
		}

		// Jeśli robot nie znajduje się w stanie STOPPED zmień prędkość silników
		if(state != STOPPED)
		{
			DRV8834_SetSpeed(&leftMotor, -speedLeft);	// prędkość z "minusem", bo silniki fizycznie zamontowane są w odbiciu lustrzanym
			DRV8834_SetSpeed(&rightMotor, speedRight);
		}

		// Prześlij przez UART aktualną wartość prędkości
		sprintf(cmd, "SPD=%d\r\n", -(int)speed);
		sendMessageToUART(cmd);
		// Prześlij przez UART aktualny stan robota co 25 iteracji algorytmu
		statePrintDelay++;
		if(statePrintDelay >= 25)
		{
			sprintf(cmd, "ST=%d\r\n", (int) state);
			sendMessageToUART(cmd);
			statePrintDelay = 0;
		}

	}

	// Sprawdzenie czy otrzymano jakieś flagi (zapytania lub rozkazy zapisu do zmiennej)
	if(osThreadFlagsWait(MOT_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < MOT_FLAGS)
	{
	uint16_t tmp;
	uint8_t isSetFlag;
	// Pobranie otrzymanych flag
	flags = osThreadFlagsGet();

	// Sprawdzenie czy otrzymano flagę zapisu do danej zmiennej
	if((isSetFlag = CHECK_BIT(flags, PARSER_MOT_SET_FLAG)))
	{
		// Jeśli tak to pobierz wartość z kolejki
		osMessageQueueGet(QueueUartMOTHandle, &tmp, 0, 0);
		// Wyczyść flagę zapisu globalnie i w lokalnej zmiennej
		osThreadFlagsClear(PARSER_MOT_SET_FLAG);
		CLEAR_BIT(flags, PARSER_MOT_SET_FLAG);
	}

	// Sprawdź której zmiennej dotyczy flaga
	// Jeśli jest to komenda zapisu to przypisz wartość z kolejki do danej zmiennej
	// Jeśli jest to komenda odczytu to wyślij przez UART wartość tej zmiennej
	switch(flags)
	{
	// Jeśli otrzymano komendę jazdy lub obrotu w danym kierunku lub zaprzestania takiej czynności
	// to zmień odpowiednio stan robota
	case PARSER_MOT_FRWD_FLAG:
		if (isSetFlag) DRV8834_setRobotState((tmp > 0.5 ? FORWARD : BALANCING));	// 1 = FORWARD, 0 = BALANCING
		break;
	case PARSER_MOT_BKWD_FLAG:
		if (isSetFlag) DRV8834_setRobotState((tmp > 0.5 ? BACKWARD : BALANCING));	// analogicznie j.w.
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
		{ 	// Jeśli mikrokroki dla silników jakimś trafem różnią się to zwróć błąd przez UART
			if(leftMotor.microstep == rightMotor.microstep)
				sprintf(cmd, "MST=%d\r\n", leftMotor.microstep);
			else
				sprintf(cmd, "#ERROR\r\n");
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
	case PARSER_MOT_TURN_SPEED_FLAG:
		if (isSetFlag) robotTurningSpeed = tmp;
		else { sprintf(cmd, "TS=%d\r\n", (int) robotTurningSpeed); sendMessageToUART(cmd); }
		break;
	case PARSER_MOT_BLOCK_FLAG:
		if (isSetFlag)	// tylko zapis, jest to komenda z programu użytkownika aby zablokować lub odblokować silniki
		{
			if(DRV8834_getRobotState() != STOPPED && ((uint8_t) tmp == 1))
				{ DRV8834_StopMotor(&leftMotor); DRV8834_StopMotor(&rightMotor); }
			else if(((uint8_t) tmp == 0))	// Jeśli otrzymano BL=0 to wystartuj silniki z ostatnią obliczoną prędkością
				{ DRV8834_StartMotor(&leftMotor, speedLeft); DRV8834_StartMotor(&rightMotor, speedRight); }
		}
		break;	// zmienna wysyłana cyklicznie, lub na żądanie (read only)
	case PARSER_MOT_SPEED_FLAG:
		if (!isSetFlag) { sprintf(cmd, "SPD=%d\r\n", (int)speed); sendMessageToUART(cmd); }
		break;
	case PARSER_MOT_STATE_FLAG:	// zmienna wysyłana cyklicznie, lub na żądanie (read only)
		if (!isSetFlag) { sprintf(cmd, "ST=%d\r\n", (int)DRV8834_getRobotState()); sendMessageToUART(cmd); }
		break;
	default: break;
	}
	// Wyczyść flagę i odblokuj mutex umożliwiając ustawienie kolejnej flagi przez parser
	osThreadFlagsClear(flags);
	osSemaphoreRelease(SemaphoreFlagsMOTHandle);
	}
  }
  /* USER CODE END StartStepperMotorsTask */
}

/* USER CODE BEGIN Header_StartOtherTask */
/**
* @brief Task, który realizuje pomniejsze funkcjonalności (np. pomiar napięcia baterii)
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOtherTask */
void StartOtherTask(void *argument)
{
  /* USER CODE BEGIN StartOtherTask */
	uint16_t batteryVoltageReadings[OTH_BAT_NUM_OF_READINGS];	// Tablica na próbki pomiaru napięcia baterii przez ADC
	float batteryVoltageMin = OTH_BAT_DEFAULT_MIN_V;	// minimalny poziom napięcia baterii (zeskalowany do zakresu od 0 do OTH_ADC_VDDA_VALUE)
	// maksymalny poziom baterii zeskalowany analogicznie i ograniczony od góry przez OTH_ADC_VDDA_VALUE
	float batteryVoltageMax = ((OTH_BAT_DEFAULT_MAX_V > OTH_ADC_VDDA_VALUE) ? OTH_ADC_VDDA_VALUE : OTH_BAT_DEFAULT_MAX_V);
	uint32_t batteryVoltageMean = 0;	// zmienna pomocnicza do obliczenia średniej z próbek
	uint8_t batteryLevel = 0;	// przechowuje aktualny poziom baterii w %
	uint16_t flags;	// Zmienna pomocnicza dla flag przesyłanych do zadania przez parser
	char cmd[16];	// Buffer pomocniczy do wysyłania wiadomości (komend) przez UART

	// Jako że semafor został utworzony z dostępnym 1 tokenem, trzeba go zabrać.
	// Semafor jest zwalniany gdy DMA zbierze odpowiednią liczbę próbek od ADC.
	osSemaphoreAcquire(SemaphoreADC_DataReadyHandle, osWaitForever);
	// Wystartowanie timera, który po odliczeniu 1s wywoła wykonanie pomiaru przez ADC
	HAL_TIM_Base_Start(&htim3);
	// Wystartowanie ADC z DMA. DMA po zebraniu OTH_BAT_NUM_OF_READINGS próbek, wywoła przerwanie
	// które zwolni semafor i wykonają się komendy z pętli nieskończonej tego tasku.
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)batteryVoltageReadings, OTH_BAT_NUM_OF_READINGS);
  /* Infinite loop */
  for(;;)
  {
	  // Czekaj aż DMA zbierze wszystkie próbki
	  if(osSemaphoreAcquire(SemaphoreADC_DataReadyHandle, 200*KERNEL_TICKS_FOR_MS) == osOK)
	  {
		  float tmp;
		  batteryVoltageMean = 0;	// zeruj średnią z pomiarów (z wartości z poprzedniej iteracji)
		  // Oblicz średnią z zebranych próbek
		  for(uint8_t i=0; i<OTH_BAT_NUM_OF_READINGS; i++)
		  {
			  batteryVoltageMean += batteryVoltageReadings[i];
		  }
		  batteryVoltageMean /= OTH_BAT_NUM_OF_READINGS;

		  // Zeskaluj surową średnią wartość uzyskaną z ADC do zakresu napięciowego ADC
		  tmp = map(batteryVoltageMean, 0, OTH_ADC_MAX_VALUE, 0, OTH_ADC_VDDA_VALUE);
		  // Jeśli zeskalowana wartość jest mniejsza od min ustawionej wartości baterii to sprowadź ją do batteryVoltageMin
		  if(tmp < batteryVoltageMin)
			  tmp = batteryVoltageMin;
		  // Oblicz poziom baterii w % skalując poziom napięcia na ADC do zakresu 0-100
		  batteryLevel = (uint8_t) map(tmp, batteryVoltageMin, batteryVoltageMax, 0, 100);

		  // Wyślij aktualny poziom baterii przez UART
		  sprintf(cmd, "BL=%d\r\n", (int) batteryLevel);
		  sendMessageToUART(cmd);
	  }

	  // Sprawdzenie czy otrzymano jakieś flagi (zapytania lub rozkazy zapisu do zmiennej)
	  if(osThreadFlagsWait(PID_FLAGS, osFlagsWaitAny | osFlagsNoClear, 0) < PID_FLAGS)
	  {
		float tmp;
		uint8_t isSetFlag;
		// Pobranie otrzymanych flag
		flags = osThreadFlagsGet();

		// Sprawdzenie czy otrzymano flagę zapisu do danej zmiennej
		if((isSetFlag = CHECK_BIT(flags, PARSER_OTH_SET_FLAG)))
		{
			osMessageQueueGet(QueueUartOTHHandle, &tmp, 0, 0);
			osThreadFlagsClear(PARSER_OTH_SET_FLAG);
			CLEAR_BIT(flags, PARSER_OTH_SET_FLAG);
		}

		// Sprawdź której zmiennej dotyczy flaga
		// Jeśli jest to komenda zapisu to przypisz wartość z kolejki do danej zmiennej
		// Jeśli jest to komenda odczytu to wyślij przez UART wartość tej zmiennej
		switch(flags)
		{
		case PARSER_OTH_BAT_LEVL_FLAG: // zmienna wysyłana cyklicznie i na żądanie (read only)
			if (!isSetFlag) { sprintf(cmd, "BL=%d\r\n", (int) batteryLevel); sendMessageToUART(cmd); }
			break;
		case PARSER_OTH_BAT_MINV_FLAG:
			if (isSetFlag) batteryVoltageMin = tmp;
			else { sprintf(cmd, "BMN=%.2f\r\n", batteryVoltageMin); sendMessageToUART(cmd); }
			break;
		case PARSER_OTH_BAT_MAXV_FLAG:
			if (isSetFlag) batteryVoltageMax = tmp;
			else {
				sprintf(cmd, "BMX=%.2f\r\n", batteryVoltageMax);
				sendMessageToUART(cmd); }
			break;
		default: break;
		}
		// Wyczyść flagę i odblokuj mutex umożliwiając ustawienie kolejnej flagi przez parser
		osThreadFlagsClear(flags);
		osSemaphoreRelease(SemaphoreFlagsOTHHandle);
	  }
  }
  /* USER CODE END StartOtherTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// ********************* SEKCJA PRZERWAŃ ********************* //

// Przerwania zewnętrzne
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Przerwanie generowane gdy pomiar z akcelerometru jest gotowy
	if(GPIO_Pin == LSM6_Interrupt_Pin)
	{
		// Zwolnienie semafora odblokowujące zadanie odczytu pomiaru z IMU
		osSemaphoreRelease(SemaphoreLSM6_DataReadyHandle);
	}
	// Przerwanie generowane gdy pomiar z żyroskopu jest gotowy
#ifdef LSM6_Interrupt2_Pin
	if(GPIO_Pin == LSM6_Interrupt2_Pin)
	{
		;	// W praktyce przy częstotliwości pomiaru 52Hz potrzebne jest tylko jedno prerwanie, ponieważ oba pomiary są
			// gotowe niemal w identycznym czasie. Implementacje na obu przerwaniach można wykonać na flagach (większych
			// od tych wykorzystywanych przez parser). Można spróbować też implementacji na semaforze zliczającym.
	}
#endif
}

// Przerwanie od ADC (a raczej DMA) oznaczające zebranie wszystkich próbek pomiarowych
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		// Zwolnienie semafora odblokowujące zadanie z pomiarem napięcia baterii
		osSemaphoreRelease(SemaphoreADC_DataReadyHandle);
	}
}

// Przerwanie od UART Rx (i DMA) oznaczające, że jakieś dane są dostępne do odczytu w buforze DMA
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART2)
	{
		// Zablokowanie dostępu do Rx RingBuffera mutexem
		osMutexAcquire(MutexRingBufferRxHandle, osWaitForever);
		// Kopiuj znaki (bajty) do ring buffera
		for(uint16_t i=0; i<Size; i++)
		{
			RB_Write(&Rx_RingBuffer, Tmp_Rx_Buffer[i]);
			// Jeśli wykryto znak nowej linii, zwolnij jeden token semafora zliczającego dostępne komendy
			if(Tmp_Rx_Buffer[i] == '\n')
			{
#if(USE_CUBE_MONITOR == 1)
				RxLines++;
#endif
				osSemaphoreRelease(SemRxLineAvailableHandle);
			}
		}
		// Po odbiorze paczki danych, włącz z powrotem nasłuchiwanie przez DMA i odblokuj mutex
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, Tmp_Rx_Buffer, TMP_DMAUART_BUFFER_SIZE);
		osMutexRelease(MutexRingBufferRxHandle);
	}
}

// Przerwanie od UART Tx, oznaczające że ostatni transfer danych przez DMA zakończył się
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		// Zwolnienie semafora, odblokowujące możliwość wysłania kolejnej komendy przez UART Tx
		osSemaphoreRelease(SemaphoreUART_TxCompleteHandle);
	}
}

// ******************* KONIEC SEKCJI PRZERWAŃ ******************* //

// ********************* SEKCJA CALLBACKÓW DLA BIBLIOTEK ********************* //

// Callback parsera - pobranie jednej linii (komendy) do sparsowania
Parser_StatusTypeDef Parser_ReceiveLineCallback(char* message)
{
	  uint8_t i = 0;
	  uint8_t character;

	  // Zablokuj dostęp do Rx RingBuffera
	  osMutexAcquire(MutexRingBufferRxHandle, osWaitForever);
	  // Odczytaj jedną linie z Rx RingBuffera
	  while(RB_Read(&Rx_RingBuffer, &character) == RB_OK)
	  {
		  // Jeśli napotkano znak końca linii zakończ odczytywanie
		  if(character == '\n')
		  {
			  // Zamień znak końca linii na NULL
			  *(message + i) = 0;
#if(USE_CUBE_MONITOR == 1)
			  RxLines--;
#endif
			  // Odblokuj mutex Rx RingBuffera
			  osMutexRelease(MutexRingBufferRxHandle);
			  return PARSER_OK;
		  }
		  // Jeśli nie napotkano znaku końca linii zapisuj do message kolejny znak
		  *(message + i) = character;
		  i++;
	  }
	  // Jeśli pętla zakończyła się bez napotkania znaku końca linii, to znaczy że
	  // RingBuffer jest pusty i wystąpił jakiś błąd w przesyłaniu komendy
	  osMutexRelease(MutexRingBufferRxHandle);
	  return PARSER_ERROR;
}

// Callbacki parsera - funkcje wysyłające flagi i wartości do zadań
void Parser_PID_PrintCallback(uint8_t flag, float value)
{
	// Jeśli ustawiona jest flaga zapisu to wrzuć value do kolejki
	if(CHECK_BIT(flag, PARSER_PID_SET_FLAG))
		osMessageQueuePut(QueueUartPIDHandle, &value, 0, osWaitForever);
	// Semafor blokujący ustawienie flag, jeśli poprzednie nie były jeszcze odczytane
	osSemaphoreAcquire(SemaphoreFlagsPIDHandle, osWaitForever);
	// Ustaw odpowiednie flagi w zadaniu
	osThreadFlagsSet(PIDTaskHandle, flag);
}
// Pozostałe callbacki są wykonane analogicznie, na osobnych kolejkach i zadaniach (flagach)
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

// ******************* KONIEC SEKCJI CALLBACKÓW DLA BIBLIOTEK ******************* //

// ********************* SEKCJA INNYCH FUNKCJI ********************* //

// Funkcja skalująca wartość z jednego zakresu do drugiego zakresu
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Funkcja realizująca filtr komplementarny
float complementaryFilter(float gyroX, float accAngle, float prevAngle, float gx_off)
{
	return (COMP_FILTER_GAIN * (prevAngle + ((gyroX*IMU_GYRO_SENSITIVITY + gx_off)/IMU_SAMPLE_TIME)))
			+ ((1 - COMP_FILTER_GAIN) * accAngle);
}

// Funkcja obliczająca kąt wychylenia tylko na podstawie wartości z akcelerometru
float calculateAccAngle(float accY, float accZ, float ay_off, float az_off)
{
	return (atan2((accY*IMU_ACC_SENSITIVITY + ay_off), (accZ*IMU_ACC_SENSITIVITY + az_off)) * 180.0/M_PI);
}

// Funkcja, dzięki której wszystkie taski mogą wysyłać komendy (komunikaty) przez UART
void sendMessageToUART(char* msg)
{
	// Zablokuj dostep do Tx RingBuffera mutexem
	osMutexAcquire(MutexRingBufferTxHandle, osWaitForever);
	// Pobierz długość wiadomości
	uint8_t length = strlen(msg);
	// Skopiuj ją do Tx RingBuffera
	for(uint8_t i = 0; i < length; i++)
		RB_Write(&Tx_RingBuffer, (uint8_t) *(msg+i));
	// Oblokuj dostęp do Tx RingBuffera
	osMutexRelease(MutexRingBufferTxHandle);
}

// ******************* KONIEC SEKCJI INNYCH FUNKCJI ******************* //

/* USER CODE END Application */

