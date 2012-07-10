/*
 * circularBuffer.h
 *
 *  Created on: Jun 20, 2012
 *      Author: stm32
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#define BUFFER_LENGTH 100

typedef struct CircularBuffer_s {
	int start;
	int end;
	char content[BUFFER_LENGTH];
} CircularBuffer;

void bufferInit(CircularBuffer* buf);
char bufferRemove(CircularBuffer* buf);
int bufferPut(CircularBuffer* buf, char ch);
int bufferPutString(CircularBuffer* buf, const char str[]);
int bufferIsFull(const CircularBuffer* buf);
int bufferIsEmpty(const CircularBuffer* buf);

#endif /* CIRCULARBUFFER_H_ */
