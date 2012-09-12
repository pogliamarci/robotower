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
#include "chprintf.h"

#define SERIAL_OUT_BITRATE 115200

CircularBuffer outputBuffer;

int main(void)
{
	Thread *shellThreadPointer = NULL;
	SerialConfig sd2Config = {
			SERIAL_OUT_BITRATE,
			0, USART_CR2_STOP1_BITS | USART_CR2_LINEN, 0
	};

	halInit();
	chSysInit();

	sdStart(&SD2, &sd2Config);
	chMtxInit(&spykeeLedMutex);
	bufferInit(&outputBuffer);
	resetLed();

	startLedBlinkerTreads();
	startTowersAndFactoriesThread();
	startSonarThread();
	startRfidThread();
	shellInit();

	chprintf((BaseChannel*) &SD2, "The firmware is ready!\n\r");

	/* Main application loop. Ensures that the shell is alive, and writes
	 * to the serial port the (whole) content of the output buffer */
	while (TRUE)
	{
		shellInitControl(&shellThreadPointer);
		writeContentOnBaseChannel(&outputBuffer, (BaseChannel*) &SD2);
	}
}
