/*
 * circularBuffer.h
 *
 *  Created on: Jun 20, 2012
 *      Author: stm32
 */

#ifndef CIRCULARBUFFER_H_
#define CIRCULARBUFFER_H_

#include "ch.h"
#include "hal.h"

#define BUFFER_LENGTH 150

/* A fixed-length circular buffer, used to synchronize writes onto a BaseChannel,
 * usually the serial port. Reads and writes to the buffer are synchronised
 * using the mutex contained in the struct.
 * NOTE: the synchronisation is automatically performed when using the functions
 * contained in this header to read from\write to the buffer. */
typedef struct CircularBuffer_s {
	int start;
	int end;
	char content[BUFFER_LENGTH];
	Mutex mutex;
} CircularBuffer;

void bufferInit(CircularBuffer* buf);
int bufferPutString(CircularBuffer* buf, const char str[]);
void writeContentOnBaseChannel(CircularBuffer* buf, BaseChannel* bc);

#endif /* CIRCULARBUFFER_H_ */
