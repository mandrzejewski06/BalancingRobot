/*
 * parser.c
 *
 *  Created on: Apr 25, 2022
 *      Author: mand2
 */
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "parser.h"
// Rozmiar wewnętrznego buffera dla jednej linii
#define BUFFER_SIZE 32
// Wskaźniki na funkcje piszące flagi i wartości do odpowiednich tasków.
// Przypisanie wskaźnika do funkcji realizują funkcje parer_register_xxx
static void (*Parser_PID_Print)(uint8_t flag, float value);
static void (*Parser_IMU_Print)(uint8_t flag, float value);
static void (*Parser_Motors_Print)(uint16_t flag, uint16_t value);
static void (*Parser_Other_Print)(uint8_t flag, float value);
// Wskaźnik na funkcję realizującą pobranie jednej linii z buffera
static Parser_StatusTypeDef (*Parser_ReceiveLine)(char* message);
// Wewnętrzny buffer dla jednej linii pobranej używając *Parser_ReceiveLine
static char BufferReceive[BUFFER_SIZE];

// Rejestrowanie callbacków
void Parser_Register_PIDPrintCallback(void *callback)
{
	Parser_PID_Print = callback;
}

void Parser_Register_IMUPrintCallback(void *callback)
{
	Parser_IMU_Print = callback;
}

void Parser_Register_MotorsPrintCallback(void *callback)
{
	Parser_Motors_Print = callback;
}

void Parser_Register_OtherPrintCallback(void *callback)
{
	Parser_Other_Print = callback;
}

void Parser_Register_ReceiveLineCallback(void *callback)
{
	Parser_ReceiveLine = callback;
}

// Wewnętrzna funkcja wyciągająca wartość float ze stringa
static Parser_StatusTypeDef Parser_GetFloatValue(float *value)
{
	// Pobranie do końca parsowanej linii
	char *ParsePointer = strtok(NULL, "\0");
	uint8_t i = 0;
	//Jeśli długość parsowanej linii wynosi 0 to znaczy że wykryto funkcję odczytu wartości
	if(strlen(ParsePointer) == 0) return PARSER_READ;
	// Wykrycie liczby ujemnej
	if(ParsePointer[0] == '-') i++;
	// Jeśli kolejne znaki nie są cyframi lub kropką zwróć błąd
	for( ; ParsePointer[i] != 0; i++)
	{
		if((ParsePointer[i] < '0' || ParsePointer[i] > '9') && (ParsePointer[i] != '.'))
		{
			return PARSER_ERROR;
		}
	}
	// Przekształć string na postać float
	*value = atof(ParsePointer);
	// Zwróć informację, że należy nadpisać parsowaną zmienną wartością *value
	return PARSER_SET;
}

// Wewnętrzna funkcja wyciagająca wartość int ze stringa
// Działa analogicznie jak funkcja powyżej
static Parser_StatusTypeDef Parser_GetIntValue(uint16_t *value)
{
	char *ParsePointer = strtok(NULL, "\0");
	uint8_t i = 0;

	if(strlen(ParsePointer) == 0) return PARSER_READ;
	if(ParsePointer[0] == '-') i++;

	for( ; ParsePointer[i] != 0; i++)
	{
		if((ParsePointer[i] < '0' || ParsePointer[i] > '9'))
		{
			return PARSER_ERROR;
		}
	}

	*value = atoi(ParsePointer);

	return PARSER_SET;
}

// Wewnętrzna funkcja parsująca komendę dla tasku z pomniejszymi funkcjonalnościami
static Parser_StatusTypeDef Parser_ParseOther(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	float value = 0;
	char *ParsePointer = strtok(NULL, "=?");
	// Sprawdzenie czy komenda ma zerową długość,
	// jesli tak zwróć błąd.
	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;

	// Sprawdzenie po kolei każdej możliwej komendy
	// i przypisanie odpowiedniej flagi zgodnej z komendą
	if(strcmp(ParsePointer, "BMX") == 0)	// Max poziom baterii
	{
		flag = PARSER_OTH_BAT_MAXV_FLAG;
	}
	else if(strcmp(ParsePointer, "BMN") == 0)	// Min poziom baterii
	{
		flag = PARSER_OTH_BAT_MINV_FLAG;
	}
	else if(strcmp(ParsePointer, "BL") == 0)	// Poziom baterii w % (read only)
	{
		flag = PARSER_OTH_BAT_LEVL_FLAG;
	}
	else return PARSER_ERROR;	// Jeśli nie wykryto żadnej komendy, zwróć błąd
	// Sprawdź czy komenda zawiera wartość do zapisu czy jest to komenda odczytu
	status = Parser_GetFloatValue(&value);
	// Sprawdź czy powyższa funkcja nie zwróciła błędu
	if(status == PARSER_ERROR) return PARSER_ERROR;
	// Sprawdź czy wykryto komende zapisu do zmiennej i ustaw flagę zapisu
	else if(status == PARSER_SET) flag |= PARSER_OTH_SET_FLAG;
	// Przekaż odpowiednią flagę i jeśli to zapis to także wartość do odpowiedniego tasku
	Parser_Other_Print(flag, value);
	return PARSER_OK;
}

// Wewnętrzna funkcja parsująca komendę dla tasku sterującego silnikami krokowymi
// Działanie funkcji jest analogiczne jak powyższej
static Parser_StatusTypeDef Parser_ParseMotors(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	uint16_t value = 0;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;

	if(strcmp(ParsePointer, "FWD") == 0)	// FWD=1 jazda prosto
	{										// FWD=0 wyłączenie jazdy prosto
		flag = PARSER_MOT_FRWD_FLAG;
	}
	else if(strcmp(ParsePointer, "BCK") == 0)	// BCK=1 jazda do tyłu
	{											// BCK=0 wyłączenie jazdy do tyłu
		flag = PARSER_MOT_BKWD_FLAG;
	}
	else if(strcmp(ParsePointer, "LFT") == 0)	// LFT=1 obrót w lewo
	{											// BCK=0 wyłączenie obrotu w lewo
		flag = PARSER_MOT_LEFT_FLAG;
	}
	else if(strcmp(ParsePointer, "RGT") == 0)	// RGT=1 obrót w prawo
	{											// RGT=0 wyłączenie obrotu w prawo
		flag = PARSER_MOT_RGHT_FLAG;
	}
	else if(strcmp(ParsePointer, "MST") == 0)	// Microstep (mikrokrok) silnika
	{
		flag = PARSER_MOT_MICROSTEP_FLAG;
	}
	else if(strcmp(ParsePointer, "MF") == 0)	// Max częstotliwość impulsów
	{											// (Częstotliwość impulsów timera)
		flag = PARSER_MOT_MAXFREQ_FLAG;
	}
	else if(strcmp(ParsePointer, "MSP") == 0)	// Max prędkość silników
	{
		flag = PARSER_MOT_MAXSPED_FLAG;
	}
	else if(strcmp(ParsePointer, "BLK") == 0) 	// BLK=1 zatrzymanie silników
	{											// (wyłączenie timera)
		flag = PARSER_MOT_BLOCK_FLAG;			// BLK=0 wystartowanie silników
	}
	else if(strcmp(ParsePointer, "SPD") == 0)	// Aktualna prędkość silników
	{											// (read only)
		flag = PARSER_MOT_SPEED_FLAG;
	}
	else if(strcmp(ParsePointer, "ST") == 0)	// Aktualny stan robota (read only)
	{
		flag = PARSER_MOT_STATE_FLAG;
	}
	else if(strcmp(ParsePointer, "TS") == 0)	// Szybkość robota podczas obrotu
	{
		flag = PARSER_MOT_TURN_SPEED_FLAG;
	}
	else return PARSER_ERROR;

	status = Parser_GetIntValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_MOT_SET_FLAG;

	Parser_Motors_Print(flag, value);
	return PARSER_OK;
}

// Wewnętrzna funkcja parsująca komendę dla tasku IMU z akcelerometrem i żyroskopem
// Działanie funkcji jest analogiczne jak powyższych
static Parser_StatusTypeDef Parser_ParseIMU(void)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	float value = 0;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;

	if(strcmp(ParsePointer, "GX") == 0)		// Offset żyroskopu na osi X
	{
		flag = PARSER_IMU_GX_OFFSET_FLAG;
	}
	else if(strcmp(ParsePointer, "AY") == 0)	// Offset akcelerometru na osi Y
	{
		flag = PARSER_IMU_AY_OFFSET_FLAG;
	}
	else if(strcmp(ParsePointer, "AZ") == 0)	// Offset akcelerometru na osi Z
	{
		flag = PARSER_IMU_AZ_OFFSET_FLAG;
	}
	else if(strcmp(ParsePointer, "ANG") == 0)	// Aktualny kąt wychylenia (read only)
	{
		flag = PARSER_IMU_ANGLE_FLAG;
	}
	else if(strcmp(ParsePointer, "CF") == 0)	// Współczynnik filtru komplementarnego
	{
		flag = PARSER_IMU_COMP_FILTER_FLAG;
	}
	else if(strcmp(ParsePointer, "CB") == 0)	// CB=1 wykonaj autokalibrację
	{											// CB=0 autokalibracja zakończona
		flag = PARSER_IMU_CALIBRATION_FLAG;		// CB=0 zwraca robot po poprawnej kalibracji
	}
	else return PARSER_ERROR;

	status = Parser_GetFloatValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_IMU_SET_FLAG;

	Parser_IMU_Print(flag, value);
	return PARSER_OK;
}

// Wewnętrzna funkcja parsująca komendę dla tasku PID
// Działanie funkcji jest analogiczne jak powyższych
// Różni się przyjmowaniem argumentu numeru regulatora
// pidNumber==1 regulator kąta wychylenia, ==2 regulator prędkości
static Parser_StatusTypeDef Parser_ParsePID(uint8_t pidNumber)
{
	Parser_StatusTypeDef status;
	uint16_t flag;
	float value;
	char *ParsePointer = strtok(NULL, "=?");

	if(strlen(ParsePointer) <= 0) return PARSER_ERROR;

	if(strcmp(ParsePointer, "KP") == 0)		// Wzmocnienie członu proporcjonalnego Kp
	{
		flag = PARSER_PID_KP_FLAG;
	}
	else if(strcmp(ParsePointer, "KI") == 0)	// Wzmocnienie członu całkującego Ki
	{
		flag = PARSER_PID_KI_FLAG;
	}
	else if(strcmp(ParsePointer, "KD") == 0)	// Wzmocnienie członu różniczkującego Kd
	{
		flag = PARSER_PID_KD_FLAG;
	}
	else if(strcmp(ParsePointer, "RML") == 0)	// Max pochylenie robota, które może
	{											// zadać regulator sterujący w funkcji
		flag = PARSER_PID_MAX_LEAN_FLAG;		// prędkości
	}
	else if(strcmp(ParsePointer, "RMS") == 0)	// Prędkość z jaką docelowo ma jechać
	{											// robot do przodu i do tyłu
		flag = PARSER_PID_MOV_SPEED_FLAG;
	}
	else if(strcmp(ParsePointer, "ZH") == 0)	// Histereza zerowej prędkości
	{											// Jeśli będzie ustawiona np. na 5
		flag = PARSER_PID_ZERO_HYST_FLAG;		// to prędkość robota od -5 do 5
	}											// będzie sprowadzona do 0
	else return PARSER_ERROR;

	status = Parser_GetFloatValue(&value);

	if(status == PARSER_ERROR) return PARSER_ERROR;
	else if(status == PARSER_SET) flag |= PARSER_PID_SET_FLAG;
	// Sprawdzenie do którego regulatora dana komenda się odnosi
	if(pidNumber == 2)
		flag |= PARSER_PID_PID2_FLAG;

	Parser_PID_Print(flag, value);
	return PARSER_OK;
}

// Główna funkcja, którą wywołuje użytkownik aby sparsować komendę (jedną linie)
Parser_StatusTypeDef Parser_ParseLine(void)
{
	char *ParsePointer;
	Parser_StatusTypeDef status;
	// Pobranie jednej linii z buffera
	// Jeśli nie można tego zrobić zwróć błąd
	if(!Parser_ReceiveLine(BufferReceive))
	{
		ParsePointer = strtok(BufferReceive, "+");
		// Sprawdzenie do którego tasku odnosi się komenda
		if(strcmp(ParsePointer, "P") == 0)	// Task PID
		{
			ParsePointer = strtok(NULL, "+");
			// Sprawdzenie do którego regulatora odnosi się komenda
			if(strcmp(ParsePointer, "1") == 0)	// 1 - regulator prędkości
				status = Parser_ParsePID(1);
			else if(strcmp(ParsePointer, "2") == 0)	// 2 - regulator kąta wychylenia
				status = Parser_ParsePID(2);
			else
				status = PARSER_ERROR;
		}
		else if(strcmp(ParsePointer, "I") == 0) // Task IMU
		{
			status = Parser_ParseIMU();
		}
		else if(strcmp(ParsePointer, "M") == 0)	// Task silników krokowych
		{
			status = Parser_ParseMotors();
		}
		else if(strcmp(ParsePointer, "O") == 0)	// Task pomniejszych funkcjonalności
		{
			status = Parser_ParseOther();
		}
		else	// Jeśli nie tyczy się żadnego z dostepnych tasków zwróć błąd
		{
			return PARSER_ERROR;
		}
		// Zwróć status wykonania wewnętrznych parserów (np ParsePID itp.)
		return status;
	}
	return PARSER_ERROR;
}
