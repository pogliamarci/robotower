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

CircularBuffer circularBuffer;

EventSource eventSource;

int main(void)
{
	Thread *shellTp = NULL;
	SerialConfig sd2Config = {
			SERIAL_OUT_BITRATE,
			0,
			USART_CR2_STOP1_BITS | USART_CR2_LINEN,
			0
	};

	/* hardware initialization */
	halInit();
	chSysInit();
	sdStart(&SD2, &sd2Config);

	chMtxInit(&spykeeLedMutex);
	bufferInit(&circularBuffer);
	chEvtInit(&eventSource);

	resetLed(); 		/* reset the leds on the robot shoulders */
	palClearPad(IOPORT4, GPIOD_IRLED);	/* ensure the IR led are turned off */

	/* thread initialization */
	startLedTreads();
	startTowersAndFactoriesThread();
	startSonarThread();
	startRfidThread();
	shellInit();

	/* Firmware start message*/
	chprintf((BaseChannel*) &SD2, "The firmware is ready!\n\r");

	/* main application loop. Ensures that the shell is alive*/
	while (TRUE)
	{
		shellInitControl(&shellTp);
		writeContentOnBaseChannel(&circularBuffer, (BaseChannel*) &SD2);
	}
}
