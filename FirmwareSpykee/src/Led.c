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

#include "FirmwareSpikee.h"

Mutex spykeeLedMutex;

static WORKING_AREA(blinkerWorkingArea, 128);
static WORKING_AREA(spykeeLedBlinkerWorkingArea, 128);

/* Led Management */
// red leds: PE7 - PE8 - PE9 - PE10
// yellow leds: PE11 - PE12 - PE13 - PE14
// green led: PE15
void setLed(int n, bool_t setOn)
{
	if (n < 0 || n > 8)
		return; /* ensure the passed led ID is right... */
	const short firstPort = 7; //using GPIOs from PE7
	if (setOn)
	{
		palSetPad(IOPORT5, n + firstPort);
	}
	else
	{
		palClearPad(IOPORT5, n + firstPort);
	}
}

/* Led Initialization */
void resetLed(void)
{
	const int numLed = 9;
	int i;
	for (i = 0; i < numLed; i++)
	{
		setLed(i, FALSE);
	}
}

/* Thread that blink sequentially the four integrated leds of the STM32F4Discovery */
static msg_t spykeeLedBlinkerThread(void *arg)
{
	short redStat = 0;
	short yellowStat = 0;
	bool_t greenStat = FALSE;

	(void) arg;
	while (TRUE)
	{
		chMtxLock(&spykeeLedMutex);
		if (blinking[0])
		{
			setLed(redStat, FALSE);
			redStat = redStat == 3 ? 1 : redStat + 1;
			setLed(redStat, TRUE);
		}
		if (blinking[1])
		{
			setLed(4 + yellowStat, FALSE);
			yellowStat = yellowStat == 3 ? 1 : yellowStat + 1;
			setLed(yellowStat, TRUE);
		}
		if (blinking[2])
		{
			setLed(8, greenStat);
			greenStat = !greenStat;
		}
		chMtxUnlock();
		chThdSleepMilliseconds(500);
	}
	return 0;
}

/* Thread that blink sequentially the four integrated leds of the STM32F4Discovery */
static msg_t blinkerThread(void *arg)
{
	(void) arg;
	while (TRUE)
	{
		palSetPad(IOPORT4, GPIOD_LED3);
		palClearPad(IOPORT4, GPIOD_LED4);
		chThdSleepMilliseconds(500);
		palSetPad(IOPORT4, GPIOD_LED5);
		palClearPad(IOPORT4, GPIOD_LED3);
		chThdSleepMilliseconds(500);
		palSetPad(IOPORT4, GPIOD_LED6);
		palClearPad(IOPORT4, GPIOD_LED5);
		chThdSleepMilliseconds(500);
		palSetPad(IOPORT4, GPIOD_LED4);
		palClearPad(IOPORT4, GPIOD_LED6);
		chThdSleepMilliseconds(500);
	}
	return 0;
}

void startLedTreads(void)
{
	chThdCreateStatic(spykeeLedBlinkerWorkingArea,
			sizeof(spykeeLedBlinkerWorkingArea), NORMALPRIO,
			spykeeLedBlinkerThread, NULL );
	chThdCreateStatic(blinkerWorkingArea, sizeof(blinkerWorkingArea),
			NORMALPRIO, blinkerThread, NULL );
}

