/*
 * Firmware for the robot Spykee, for the STM32F4xx board
 *
 * Copyright (C) 2012 Politecnico di Milano
 * Copyright (C) 2012 Marcello Pogliani, Davide Tateo
 * Versione 1.0
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "FirmwareSpykee.h"

static WORKING_AREA(rfidWorkingArea, 1024);

/* Thread used to manage rfid Reader*/
static msg_t rfidThread(void *arg)
{
	(void) arg;
	const int rfidMessageSize = 16;
	const int rfidBitrate = 9600;

	SerialConfig config =
	{ rfidBitrate, 0, USART_CR2_STOP1_BITS | USART_CR2_LINEN, 0 };
	char buf[rfidMessageSize + 1];
	char buf2[rfidMessageSize + 9];

	sdStart(&SD3, &config);
	while (TRUE)
	{
		sdRead(&SD3, (uint8_t*) buf, sizeof(buf)-1);
		/* Specification of the ID-12 output data format (ASCII):
		 * STX (0x02) | DATA (10 ASCII chars) | CHECKSUM (2 ASCII) | CR | LF | ETX (0x03)
		 * We transmit from here only the data and the checksum. The checksum is not checked
		 * on board, but will be checked on the computer. The checksum is the xor of
		 * the 5 hex bytes (10 ascii) DATA characters.
		 */
		/* this is rather ugly (using two buffers...) but for now it works as expected */
		buf[rfidMessageSize - 3] = '\0'; // strip the trailing CR, LF, ETX
		chsprintf(buf2, "[RFID] %s", buf + 1); // +1 to strip the leading STX char

		bufferPutString(&outputBuffer, buf2);
	}
	return 0;
}

void startRfidThread(void)
{
	chThdCreateStatic(rfidWorkingArea, sizeof(rfidWorkingArea), NORMALPRIO,
			rfidThread, NULL );
}

