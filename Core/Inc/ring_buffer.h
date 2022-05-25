/*
 * ring_buffer.h
 *
 *  Created on: Dec 25, 2020
 *      Author: mand2
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>
// Rozmiar ring buffera w bajtach
#define RING_BUFFER_SIZE 200
// Zmienna zwracana przez funkcje biblioteki (błąd lub ok)
typedef enum {
	RB_OK = 0,
	RB_ERROR = 1
} RB_StatusTypeDef;
// Struktura obiektu ring buffera
typedef struct {
	uint16_t Head;	// głowa czyli miejsce kolejnego zapisu buffera
	uint16_t Tail;	// ogon czyli miejsce ostatniego odczytu buffera
	uint8_t Buffer[RING_BUFFER_SIZE]; // tablica przechowująca wartości bajtowe
} RingBuffer_t;
// Funkcja sprawdzająca czy jest coś do oczytu z buffera
RB_StatusTypeDef isReadable(RingBuffer_t *rb);
// Funkcja odczytująca jeden bajt do zmiennej value
RB_StatusTypeDef RB_Read(RingBuffer_t *rb, uint8_t *Value);
// Funkcja zapisująca jeden bajt do ring buffera
RB_StatusTypeDef RB_Write(RingBuffer_t *rb, uint8_t Value);

#endif /* INC_RING_BUFFER_H_ */

