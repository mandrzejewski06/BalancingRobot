/*
 * parser.h
 *
 *  Created on: Apr 25, 2022
 *      Author: mand2
 */

#ifndef INC_PARSER_H_
#define INC_PARSER_H_

#include <stdint.h>
// Flagi, które precyzują którą zmienną odczytał parser
// FLAGI DO PID
#define PARSER_PID_KP_FLAG 			0x01	// 0b0000 0001
#define PARSER_PID_KI_FLAG			0x02	// 0b0000 0010
#define PARSER_PID_KD_FLAG			0x04	// itd...
// Jeśli poniższa flaga=0 to powyższe nastawy dotyczą PID1
// A jeżeli flaga=1 to dotyczą PID2
#define PARSER_PID_PID2_FLAG		0x08
#define PARSER_PID_MAX_LEAN_FLAG	0x10
#define PARSER_PID_MOV_SPEED_FLAG	0x20
#define PARSER_PID_ZERO_HYST_FLAG	0x40
// Poniższa flaga jeśli jest równa 1 to oznacza, że odczytano
// komende zapisu zmiennej, jeśli jest równa 0 to odczytano
// komende odczytu zmiennej i należy odesłać jej wartość
#define PARSER_PID_SET_FLAG			0x80

// FLAGI DO IMU
#define PARSER_IMU_GX_OFFSET_FLAG	0x01
#define PARSER_IMU_AY_OFFSET_FLAG	0x02
#define PARSER_IMU_AZ_OFFSET_FLAG	0x04
// Zmienna angle jest wysyłana cyklicznie, ale jest też
// możliwość jej wymuszonego odczytu. Nie można do niej nic zapisać
#define PARSER_IMU_ANGLE_FLAG		0x08
#define PARSER_IMU_CALIBRATION_FLAG	0x10 // flaga komendy wymuszającej autokalibracje
#define PARSER_IMU_COMP_FILTER_FLAG	0x20
#define PARSER_IMU_SET_FLAG			0x80

// FLAGI DO SILNIKÓW KROKOWYCH
// Poniżej flagi zadawania kierunku jazdy robota
// Flaga = 1 oznacza zmiane stanu robota zgodny z daną flagą,
// natomiast 0 oznacza powrót do trybu balansowania
// Nie powinno się ustawiać 2 flag jednocześnie w stan wysoki
#define PARSER_MOT_FRWD_FLAG		0x01
#define PARSER_MOT_BKWD_FLAG		0x02
#define PARSER_MOT_LEFT_FLAG		0x04
#define PARSER_MOT_RGHT_FLAG		0x08
// Pozostałe flagi silników krokowych
#define PARSER_MOT_MICROSTEP_FLAG	0x10
#define PARSER_MOT_MAXFREQ_FLAG		0x20
#define PARSER_MOT_MAXSPED_FLAG		0x40
#define PARSER_MOT_BLOCK_FLAG		0x80
#define PARSER_MOT_SPEED_FLAG		0x100 // wysyłane cyklicznie, read only
#define PARSER_MOT_STATE_FLAG		0x200 // wysyłane cyklicznie, read only
#define PARSER_MOT_TURN_SPEED_FLAG	0x400
#define PARSER_MOT_SET_FLAG			0x800

// INNE FLAGI
#define PARSER_OTH_BAT_LEVL_FLAG	0x01 // wysyłane cyklicznie, read only
#define PARSER_OTH_BAT_MINV_FLAG	0x02
#define PARSER_OTH_BAT_MAXV_FLAG	0x04
#define PARSER_OTH_SET_FLAG			0x80

// Typ przechowujący stan parsera
typedef enum {PARSER_OK = 0, PARSER_READ, PARSER_SET, PARSER_ERROR} Parser_StatusTypeDef;
// Funkcje pomocnicze do rejestracji callbacków funkcji piszących do tasków
void Parser_Register_PIDPrintCallback(void *callback);
void Parser_Register_IMUPrintCallback(void *callback);
void Parser_Register_MotorsPrintCallback(void *callback);
void Parser_Register_OtherPrintCallback(void *callback);
// Rejestracja callbacku funkcji pobierającej jedną linie z ring buffera
void Parser_Register_ReceiveLineCallback(void *callback);
// Główną funkcja użytkownika parsująca jedną komendę z buffera
Parser_StatusTypeDef Parser_ParseLine(void);

#endif /* INC_PARSER_H_ */
