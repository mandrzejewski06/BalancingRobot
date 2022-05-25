/*
 * LSM6.h
 *
 *  Created on: Nov 25, 2021
 *      Author: mand2
 */

#ifndef INC_LSM6_H_
#define INC_LSM6_H_

#include "main.h"

// Adresy LSM6DS33 w zależności od stanu pinu SA0
#define DS33_SA0_HIGH_ADDRESS 0b1101011		// SA0 - stan wysoki
#define DS33_SA0_LOW_ADDRESS  0b1101010		// SA0 - stan niski
// Wartość rejestru WHO_AM_I (0x0F) dla układu DS33
#define DS33_WHO_ID    0x69

// Domyślna wartość dla błędu i timeout'u
// TODO: usunąć error, implementacje błędu zrobić w enumie
#define ERROR 0
#define DEFAULT_TIMEOUT 1000

// Adresy rejestrów układu LSM6DS33 - opisy w dokumentacji układu
#define      FUNC_CFG_ACCESS    0x01
#define      FIFO_CTRL1         0x06
#define   	 FIFO_CTRL2         0x07
#define      FIFO_CTRL3         0x08
#define      FIFO_CTRL4         0x09
#define      FIFO_CTRL5         0x0A
#define      ORIENT_CFG_G       0x0B

#define      INT1_CTRL          0x0D
#define      INT2_CTRL          0x0E
#define      WHO_AM_I           0x0F
#define      CTRL1_XL           0x10
#define      CTRL2_G            0x11
#define      CTRL3_C            0x12
#define      CTRL4_C            0x13
#define      CTRL5_C            0x14
#define      CTRL6_C            0x15
#define      CTRL7_G            0x16
#define      CTRL8_XL           0x17
#define      CTRL9_XL           0x18
#define      CTRL10_C           0x19

#define      WAKE_UP_SRC        0x1B
#define      TAP_SRC            0x1C
#define      D6D_SRC            0x1D
#define      STATUS_REG         0x1E

#define      OUT_TEMP_L         0x20
#define      OUT_TEMP_H         0x21
#define      OUTX_L_G           0x22
#define      OUTX_H_G           0x23
#define      OUTY_L_G           0x24
#define      OUTY_H_G           0x25
#define      OUTZ_L_G           0x26
#define      OUTZ_H_G           0x27
#define      OUTX_L_XL          0x28
#define      OUTX_H_XL          0x29
#define      OUTY_L_XL          0x2A
#define      OUTY_H_XL          0x2B
#define      OUTZ_L_XL          0x2C
#define      OUTZ_H_XL          0x2D

#define      FIFO_STATUS1       0x3A
#define      FIFO_STATUS2       0x3B
#define      FIFO_STATUS3       0x3C
#define      FIFO_STATUS4       0x3D
#define      FIFO_DATA_OUT_L    0x3E
#define      FIFO_DATA_OUT_H    0x3F
#define      TIMESTAMP0_REG     0x40
#define      TIMESTAMP1_REG     0x41
#define      TIMESTAMP2_REG     0x42

#define      STEP_TIMESTAMP_L   0x49
#define      STEP_TIMESTAMP_H   0x4A
#define      STEP_COUNTER_L     0x4B
#define      STEP_COUNTER_H     0x4C

#define      FUNC_SRC           0x53

#define      TAP_CFG            0x58
#define      TAP_THS_6D         0x59
#define      INT_DUR2           0x5A
#define      WAKE_UP_THS        0x5B
#define      WAKE_UP_DUR        0x5C
#define      FREE_FALL          0x5D
#define      MD1_CFG            0x5E
#define      MD2_CFG            0x5F

// Struktura wektora 3D
typedef struct vector
{
	double x;
	double y;
	double z;
} Vector_t;

// Wybór ręczny układu scalonego znajdującego się w module LSM6 lub wykorzystanie autodetekcji*.
// Na dzień 02.05.22r. istnieje tylko wersja z układem DS33, jednak jest to szablon przygotowany
// na przyszłe możliwe inne wersje układu.
typedef enum deviceType {device_DS33, device_autoDetect} deviceType;

// Wybór ręczny stanu pinu SA0 lub wykorzystanie autodetekcji*.
typedef enum sa0State {sa0_low, sa0_high, sa0_autoDetect} sa0State;
// *Dokładny opis o co chodzi w autodektekcji znajduje się w pliku LSM6.c

// Struktura obiektu LSM6
typedef struct lsm6
{
	I2C_HandleTypeDef *i2c;	// Wskaźnik do używanego I2C w STM32
	deviceType _device; 	// Rodzaj układu scalonego w module LSM6
	uint8_t address;		// Adres I2C modułu
	// TODO: timeout przenieść jako zmienną statyczną do pliku LSM6.c
	uint16_t io_timeout;	// Timeout dla transmisji na I2C
	// TODO: usunąć did_timeout i did_error i zaimplementować enum, który funkcje będą zwracały
	bool did_timeout;		// Czy od ostatniego sprawdzenia wystąpił timeout?
	bool did_error;			// Czy od ostatniego sprawdzenia wystąpił error?
	// Wektory przechowujące najnowszee dane z modułu
	Vector_t accelerometer; // Wektor dla akcelerometru
	Vector_t gyroscope;		// Wektor dla żyroskopu
} LSM6_t;

// Funkcja inicjalizująca moduł LSM6
// TODO: zmienić funkcje InitEx na Init
bool LSM6_InitEx(I2C_HandleTypeDef *i2c, LSM6_t *LSM6, deviceType device, sa0State sa0);
// Funkcja zwracająca wartość rejesru WHO_AM_I, zwraca 0 jeśli nie udało się odczytać
uint8_t testReg(LSM6_t *LSM6, uint8_t address, uint8_t reg);
// Funkcja wpisująca 8-bitów do danego rejestru (po I2C)
void writeReg(LSM6_t *LSM6, uint8_t reg, uint8_t value);
// Funkcja czytająca 8-bitów z danego rejestru (po I2C)
uint8_t readReg(LSM6_t *LSM6, uint8_t reg);
// Funkcja ustawiająca domyślny tryb modułu (ustawia przerwania, częstotliwość itp.)
// TODO: zaimplementować możliwość zmiany parametrów w trakcie działania programu
void enableDefault(LSM6_t *LSM6);
// Funkcja czytająca dane z akcelerometru
uint8_t readAcc(LSM6_t *LSM6);
// Funkcja czytająca dane z żyroskopu
uint8_t readGyro(LSM6_t *LSM6);
//TODO: Rozważyć przeniesienie funkcji readAcc i readGyro jako funkcji statycznych

// Główna funkcja slużąca do odczytu danych z LSM6
uint8_t LSM6_Read(LSM6_t *LSM6);
// Funkcje służące do obsługi timeout'u
// TODO: Usunąć funkcję timeoutOccured
bool timeoutOccurred(LSM6_t *LSM6);
void setTimeout(LSM6_t *LSM6, uint16_t timeout);
uint16_t getTimeout(LSM6_t *LSM6);
// Funkcja informująca czy od ostatniego sprawdzenia wystąpił error
// TODO: Usunąć tę funkcję
bool errorOccured(LSM6_t *LSM6);

#endif /* INC_LSM6_H_ */
