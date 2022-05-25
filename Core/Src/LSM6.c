/*
 * LSM6.c
 *
 *  Created on: Nov 25, 2021
 *      Author: mand2
 */

#include "LSM6.h"
#include "main.h"
#include "math.h"

// ********** FUNKCJA INICJALIZUJĄCA JEDNOSTKĘ MODUŁU LSM6 ********** //

// TODO: Dodać wykonanie funkcji enableDefault do funkcji inicjalizującej?
bool LSM6_InitEx(I2C_HandleTypeDef *i2c, LSM6_t *LSM6, deviceType device, sa0State sa0)
{
	// Inicjalizacja parametrów modułu
	LSM6->i2c = i2c;
	LSM6->_device = device;
	// TODO: usunąć poniższe zmienne, z io_timeout stworzyć globalną zmienną statyczną
	LSM6->io_timeout = DEFAULT_TIMEOUT;
	LSM6->did_timeout = false;
	LSM6->did_error = false;

	// Jeśli któryś z parametrów ustawiony jest jako autoDetect, wykonaj autodetekcję urządzenia
	if (device == device_autoDetect || sa0 == sa0_autoDetect)
	{
		// Jeśli deviceType ustawiono jako autoDetect lub sprecyzowano DS33 sprawdź czy to jest DS33
		if (device == device_autoDetect || device == device_DS33)
		{
			// Jeśli SA0 sprecyzowano jako stan wysoki lub ustawiono autodetekcję stanu, wyślij zapytanie
			// pod adres odpowiadający stanu wysokiemu SA0 i sprawdź czy w odpowiedzi ID będzie prawidłowe
			if (sa0 != sa0_low && testReg(LSM6, DS33_SA0_HIGH_ADDRESS, (uint8_t) WHO_AM_I) == DS33_WHO_ID)
			{
				// Ustaw parametry prawidłowo wykrytego urządzenia
				sa0 = sa0_high;
				if (device == device_autoDetect) { device = device_DS33; }
			}
			// Jeśli SA0 sprecyzowano jako stan niski lub ustawiono autodetekcję stanu, wyślij zapytanie
			// pod adres odpowiadający stanu niskiemu SA0 i sprawdź czy w odpowiedzi ID będzie prawidłowe
			else if (sa0 != sa0_high && testReg(LSM6, DS33_SA0_LOW_ADDRESS, (uint8_t) WHO_AM_I) == DS33_WHO_ID)
			{
				// Ustaw parametry prawidłowo wykrytego urządzenia
				sa0 = sa0_low;
				if (device == device_autoDetect) { device = device_DS33; }
			}
		}
		/* Tutaj dodawać kolejne else if, gdyby w przyszłości wyprodukowano moduł LSM6 z innym układem scalonym
		 */

		// Jeśli w tym kroku algorytmu nie udało się znaleźć urządzenia - zwróć błąd
		if (device == device_autoDetect || sa0 == sa0_autoDetect)
		{
			return false;
		}
	}
	// Przypisz prawidłowo wykryty typ urządzenia
	LSM6->_device = device;
	// Przypisz adres do urządzenia zgodnie ze stanem pinu SA0, zwróć błąd gdyby urządzenie nie było wykryte właściwe
	switch (device)
	{
		case device_DS33:
			LSM6->address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
			break;
		default:
			return false;
	}
	// Zwróć stan OK, jeśli wszystko przebiegło pomyślnie i urządzenie jest wykryte
	return true;
}

// ********** FUNKCJA TESTUJĄCA ID URZĄDZENIA POD WSKAZANYM ADRESEM ********** //
uint8_t testReg(LSM6_t *LSM6, uint8_t address, uint8_t reg)
{
	uint8_t Value;

	// Jeśli poprawnie odczytano rejestr z ID zwróć to ID
	if (HAL_OK == HAL_I2C_Mem_Read(LSM6->i2c, (address)<<1, reg, I2C_MEMADD_SIZE_8BIT, &Value, I2C_MEMADD_SIZE_8BIT, LSM6->io_timeout))
	{
		return Value;
	}

	// Jeśli odczytanie rejestru zakończyło się błędem, zwróć error
	else
	{
		return false;
	}
}

// TODO: Zmienić implementacje funkcji timeout i error
bool timeoutOccurred(LSM6_t *LSM6)
{
	bool tmp = LSM6->did_timeout;
	LSM6->did_timeout = false;
	return tmp;
}

bool errorOccured(LSM6_t *LSM6)
{
	bool tmp = LSM6->did_error;
	LSM6->did_error = false;
	return tmp;
}

void setTimeout(LSM6_t *LSM6, uint16_t timeout)
{
	LSM6->io_timeout = timeout;
}

uint16_t getTimeout(LSM6_t *LSM6)
{
	return LSM6->io_timeout;
}

// ********** FUNKCJA USTAWIAJACA DOMYŚLNE PARAMETRY MODUŁU (WYJAŚNIONE PONIŻEJ) ********** //
void enableDefault(LSM6_t *LSM6)
{
	// Jeśli typ urządzenia to DS33, wtedy ustaw podane rejestry
	if (LSM6->_device == device_DS33)
	{
	// Dla akcelerometru
	  // Aktywuj pomiar na osiach X, Y, Z
	  writeReg(LSM6, CTRL9_XL, 0x38);	// 0x38 = 0011 1000
	  // Tryb pomiaru z częstotliwością 52Hz, zakres +-2g, antyaliasing 400Hz
	  writeReg(LSM6, CTRL1_XL, 0x30);	// 0x30 = 0011 0000
	  // Ustaw pin INT1 w stan wysoki, kiedy pomiar z akcelerometru będzie gotowy
	  writeReg(LSM6, INT1_CTRL, 0x01);	// 0x01 = 0000 0001

	// Dla żyroskopu
	  // Aktywuj pomiar na osiach X, Y, Z
	  writeReg(LSM6, CTRL10_C, 0x38);	// 0x38 = 0011 1000
	  // Tryb pomiaru z częstotliwością 52Hz, zakres +-245dps
	  writeReg(LSM6, CTRL2_G, 0x30);	// 0x30 = 0011 0000
	  // Ustaw pin INT2 w stan wysoki, kiedy pomiar z żyroskopu będzie gotowy
	  writeReg(LSM6, INT2_CTRL, 0x02);	// 0x02 = 0000 0010

	// Wspólne
	  // Brak
	}
	// Tu dodawać kolejne else if w przypadku wyprodukowania modułu z innym układem scalonym
}

// ********** FUNKCJA ZAPISUJĄCA WARTOŚĆ 8-BIT DO PODANEGO REJESTRU ********** //
void writeReg(LSM6_t *LSM6, uint8_t reg, uint8_t value)
{
	uint8_t status;

	// Prześlij 8-bitów po I2C
	status = HAL_I2C_Mem_Write(LSM6->i2c, (LSM6->address<<1), reg, I2C_MEMADD_SIZE_8BIT, &value, I2C_MEMADD_SIZE_8BIT, LSM6->io_timeout);

	// Sprawdź status transmisji
	// TODO: Przeimplementować timeout i error
	if(status == HAL_BUSY)
	{
		LSM6->did_timeout = true;
	}
	if(status == HAL_ERROR)
	{
		LSM6->did_error = true;
	}
}

// ********** FUNKCJA CZYTAJĄCA WARTOŚĆ 8-BIT Z PODANEGO REJESTRU ********** //
uint8_t readReg(LSM6_t *LSM6, uint8_t reg)
{
	uint8_t value, status;

	// Odczytaj 8-bitów po I2C
	status = HAL_I2C_Mem_Read(LSM6->i2c, (LSM6->address<<1), reg, I2C_MEMADD_SIZE_8BIT, &value, I2C_MEMADD_SIZE_8BIT, LSM6->io_timeout);

	// Sprawdź status transmisji
	// TODO: Przeimplementować timeout i error
	if(status == HAL_BUSY)
	{
		LSM6->did_timeout = true;
		return 0;
	}
	if(status == HAL_ERROR)
	{
		LSM6->did_error = true;
		return 0;
	}

	return value;
}

// ********** FUNKCJA CZYTAJĄCA OSTATNI POMIAR AKCELEROMETRU ********** //
uint8_t readAcc(LSM6_t *LSM6)
{
	// Po kolei odczytaj wartości dla każdej z osi (jest to wartość 16bit, więc odczytaj
	// najpierw bajt młodszy, potem starszy).
	uint8_t xla = readReg(LSM6, OUTX_L_XL);
	uint8_t xha = readReg(LSM6, OUTX_H_XL);
	uint8_t yla = readReg(LSM6, OUTY_L_XL);
	uint8_t yha = readReg(LSM6, OUTY_H_XL);
	uint8_t zla = readReg(LSM6, OUTZ_L_XL);
	uint8_t zha = readReg(LSM6, OUTZ_H_XL);

	// Sprawdź status transmisji
	// TODO: Przeimplementować timeout i error
	if (timeoutOccurred(LSM6) == true)
	{
		return HAL_BUSY;
	}
	if (errorOccured(LSM6) == true)
	{
		return HAL_ERROR;
	}

	// Jeśli nie było błędów złóż ze sobą bajt młodszy i starszy każdej z osi i przypisz
	// wartości do wektora akcelerometru
	LSM6->accelerometer.x = (int16_t)(xha << 8 | xla);
	LSM6->accelerometer.y = (int16_t)(yha << 8 | yla);
	LSM6->accelerometer.z = (int16_t)(zha << 8 | zla);
	// Zwróć stan OK
	return HAL_OK;
}

// ********** FUNKCJA CZYTAJĄCA OSTATNI POMIAR ŻYROSKOPU ********** //
uint8_t readGyro(LSM6_t *LSM6)
{
	// Po kolei odczytaj wartości dla każdej z osi (jest to wartość 16bit, więc odczytaj
	// najpierw bajt młodszy, potem starszy).
	uint8_t xlg = readReg(LSM6, OUTX_L_G);
	uint8_t xhg = readReg(LSM6, OUTX_H_G);
	uint8_t ylg = readReg(LSM6, OUTY_L_G);
	uint8_t yhg = readReg(LSM6, OUTY_H_G);
	uint8_t zlg = readReg(LSM6, OUTZ_L_G);
	uint8_t zhg = readReg(LSM6, OUTZ_H_G);

	// Sprawdź status transmisji
	// TODO: Przeimplementować timeout i error
	if (timeoutOccurred(LSM6) == true)
	{
		return HAL_BUSY;
	}
	if (errorOccured(LSM6) == true)
	{
		return HAL_ERROR;
	}

	// Jeśli nie było błędów złóż ze sobą bajt młodszy i starszy każdej z osi i przypisz
	// wartości do wektora żyroskopu
	LSM6->gyroscope.x = (int16_t)(xhg << 8 | xlg);
	LSM6->gyroscope.y = (int16_t)(yhg << 8 | ylg);
	LSM6->gyroscope.z = (int16_t)(zhg << 8 | zlg);
	// Zwróć stan OK
	return HAL_OK;
}

// ********** FUNKCJA CZYTAJĄCA ŻYROSKOP I AKCELEROMETR ********** //
uint8_t LSM6_Read(LSM6_t *LSM6)
{
	uint8_t status;

	// Odczytaj najpierw dane z akcelerometru
	if ((status = readAcc(LSM6)) != HAL_OK)
	{
		return status;	// Jeśli wystąpił błąd, zwróć error
	}

	// Odczytaj dane z żyroskopu
	if ((status = readGyro(LSM6)) != HAL_OK)
	{
		return status; // Jeśli wystąpił błąd, zwróć error
	}
	// Jeśli oba odczyty są poprawne, zwróć stan OK
	return HAL_OK;
}
