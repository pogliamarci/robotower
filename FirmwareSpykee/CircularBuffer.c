/*
 * CircularBuffer.c
 *
 *  Created on: Jun 20, 2012
 *      Author: stm32
 */

#include "CircularBuffer.h"
#include "ch.h"
#include "hal.h"
#include "chprintf.h"

void bufferInit(CircularBuffer* buf) {
	buf->start = 0;
	buf->end = 0;
}

char bufferRemove(CircularBuffer* buf) {
	if (bufferIsEmpty(buf)) return '\0';
	char carattere = buf->content[buf->start];
	buf->start = (buf->start + 1) % BUFFER_LENGTH;
	return carattere;
}

int bufferPut(CircularBuffer* buf, char ch) {
	if (bufferIsFull(buf))
	{
		chprintf((BaseChannel*) &SD2, "BUFFER PIENO!!! ERRORE!!!");
		return -1;
	}
	buf->content[buf->end] = ch;
	buf->end = (buf->end + 1) % BUFFER_LENGTH;
	return 0;
}

int bufferPutString(CircularBuffer* buf, const char str[]) {
	int i = 0;
	int esito = 0;
	for (i = 0; str[i] != '\0' && esito == 0; i++) {
		esito = bufferPut(buf, str[i]);
	}
	if (esito == 0) {
		bufferPut(buf, '\n');
		esito = bufferPut(buf, '\r');
	}
	return esito;
}

int bufferIsFull(const CircularBuffer* buf) {
	return (buf->end + 1) % BUFFER_LENGTH == buf->start;
}

int bufferIsEmpty(const CircularBuffer* buf) {
	return buf->start == buf->end;
}
