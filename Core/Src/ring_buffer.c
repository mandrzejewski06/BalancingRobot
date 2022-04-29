/*
 * ring_buffer.c
 *
 *  Created on: Dec 25, 2020
 *      Author: mand2
 */
#include "ring_buffer.h"

RB_StatusTypeDef isReadable(RingBuffer_t *rb)
{
	return (rb->Head == rb->Tail) ? !RB_ERROR : !RB_OK;
}

RB_StatusTypeDef RB_Read(RingBuffer_t *rb, uint8_t *Value)
{
	if (rb->Tail == rb->Head)
		{
			return RB_ERROR;
		}

	*Value = rb->Buffer[rb->Tail];
	rb->Tail = (rb->Tail +1) % RING_BUFFER_SIZE;
	return RB_OK;
}

RB_StatusTypeDef RB_Write(RingBuffer_t *rb, uint8_t Value)
{
	uint16_t TmpHead = (rb->Head +1) % RING_BUFFER_SIZE;

	if (TmpHead == rb->Tail)
	{
		return RB_ERROR;
	}

	rb->Buffer[rb->Head] = Value;
	rb->Head = TmpHead;
	return RB_OK;
}
