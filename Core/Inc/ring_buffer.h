/*
 * ring_buffer.h
 *
 *  Created on: Dec 25, 2020
 *      Author: mand2
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>

#define RING_BUFFER_SIZE 200

typedef enum {
	RB_OK = 0,
	RB_ERROR = 1
} RB_StatusTypeDef;

typedef struct {
	uint16_t Head;
	uint16_t Tail;
	uint8_t Buffer[RING_BUFFER_SIZE];
} RingBuffer_t;

RB_StatusTypeDef isReadable(RingBuffer_t *rb);
RB_StatusTypeDef RB_Read(RingBuffer_t *rb, uint8_t *Value);
RB_StatusTypeDef RB_Write(RingBuffer_t *rb, uint8_t Value);

#endif /* INC_RING_BUFFER_H_ */

