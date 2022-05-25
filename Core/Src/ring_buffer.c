/*
 * ring_buffer.c
 *
 *  Created on: Dec 25, 2020
 *      Author: mand2
 */
#include "ring_buffer.h"

// Funkcja sprawdzająca czy jest coś do oczytu z buffera
RB_StatusTypeDef isReadable(RingBuffer_t *rb)
{
	// Jeśli head == tail to nie ma nic do odczytu
	return (rb->Head == rb->Tail) ? !RB_ERROR : !RB_OK;
}
// Funkcja odczytująca jeden bajt do zmiennej value
RB_StatusTypeDef RB_Read(RingBuffer_t *rb, uint8_t *Value)
{
	// Jeśli head==tail to nie można nic odczytać więc zwróć błąd
	if (rb->Tail == rb->Head)
		{
			return RB_ERROR;
		}
	// Przypisz do zmiennej użytkownika wartość z ring buffera
	// i zwiększ Tail o jeden
	*Value = rb->Buffer[rb->Tail];
	rb->Tail = (rb->Tail +1) % RING_BUFFER_SIZE;
	return RB_OK;
}
// Funkcja zapisująca jeden bajt do ring buffera
RB_StatusTypeDef RB_Write(RingBuffer_t *rb, uint8_t Value)
{
	// Zmienna pomocnicza, żeby sprawdzić czy head nie uderzy w Tail
	uint16_t TmpHead = (rb->Head +1) % RING_BUFFER_SIZE;
	// Jeśli tak to bufor jest pełny, zwróć błąd
	if (TmpHead == rb->Tail)
	{
		return RB_ERROR;
	}
	// Jeśli nie jest pełny to zapisz wartość do ring buffera
	// i zwiększ Head o jeden
	rb->Buffer[rb->Head] = Value;
	rb->Head = TmpHead;
	return RB_OK;
}
