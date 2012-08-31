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

#define LED_FIRST_INDEX 7 // led are using GPIOs from PE7
#define LED_BLINKING_PERIOD 500 // milliseconds

Mutex spykeeLedMutex;

static WORKING_AREA(blinkerWorkingArea, 128);
static WORKING_AREA(spykeeLedBlinkerWorkingArea, 128);

/* enable the blinking mode for the three groups of leds on the robot */
bool_t blinking[] = {FALSE, FALSE, FALSE};

/* Led Management. GPIOs are triggered as follows (in order, from n=0 to n=8):
 *  --> red leds: PE7 - PE8 - PE9 - PE10
 *  --> yellow leds: PE11 - PE12 - PE13 - PE14
 *  --> green led: PE15
 */
void setLed(int n, bool_t setOn)
{
	if (n < 0 || n > 8) return;
	if (setOn)
		palSetPad(IOPORT5, n + LED_FIRST_INDEX);
	else
		palClearPad(IOPORT5, n + LED_FIRST_INDEX);
}

/* Led Initialization */
void resetLed(void)
{
	const int numLed = 9;
	short i;
	chMtxLock(&spykeeLedMutex);
	for(i = 0; i < NUM_LED_GROUPS; i++)
		blinking[i] = FALSE;
	for (i = 0; i < numLed; i++)
		setLed(i, FALSE);
	chMtxUnlock();
	palClearPad(IOPORT4, GPIOD_IRLED); /* reset infrared leds */
}

/* Blinks sequentially the leds on the robot shoulders that are on the 'blinking' status.
 * For the groups of more than one led, blinking is done by cycling sequentially the led
 * that is currently turned on, to produce a 'loading bar' effect */
static msg_t spykeeLedBlinkerThread(void *arg)
{
	short redCurrentIndex = 3;
	short yellowCurrentIndex = 3;
	bool_t greenIsOn = FALSE;

	(void) arg;
	while (TRUE)
	{
		chMtxLock(&spykeeLedMutex);
		if (blinking[0])
		{
			setLed(redCurrentIndex, FALSE);
			redCurrentIndex = redCurrentIndex == 3 ? 0 : redCurrentIndex + 1;
			setLed(redCurrentIndex, TRUE);
		}
		if (blinking[1])
		{
			setLed(4 + yellowCurrentIndex, FALSE);
			yellowCurrentIndex = yellowCurrentIndex == 3 ? 0 : yellowCurrentIndex + 1;
			setLed(4 + yellowCurrentIndex, TRUE);
		}
		if (blinking[2])
		{
			setLed(8, greenIsOn);
			greenIsOn = !greenIsOn;
		}
		chMtxUnlock();
		chThdSleepMilliseconds(LED_BLINKING_PERIOD);
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
		chThdSleepMilliseconds(LED_BLINKING_PERIOD);
		palSetPad(IOPORT4, GPIOD_LED5);
		palClearPad(IOPORT4, GPIOD_LED3);
		chThdSleepMilliseconds(LED_BLINKING_PERIOD);
		palSetPad(IOPORT4, GPIOD_LED6);
		palClearPad(IOPORT4, GPIOD_LED5);
		chThdSleepMilliseconds(LED_BLINKING_PERIOD);
		palSetPad(IOPORT4, GPIOD_LED4);
		palClearPad(IOPORT4, GPIOD_LED6);
		chThdSleepMilliseconds(LED_BLINKING_PERIOD);
	}
	return 0;
}

void startLedBlinkerTreads(void)
{
	chThdCreateStatic(spykeeLedBlinkerWorkingArea,
			sizeof(spykeeLedBlinkerWorkingArea), NORMALPRIO,
			spykeeLedBlinkerThread, NULL );
	chThdCreateStatic(blinkerWorkingArea, sizeof(blinkerWorkingArea),
			NORMALPRIO, blinkerThread, NULL );
}

