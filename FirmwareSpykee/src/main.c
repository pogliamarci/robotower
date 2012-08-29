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

CircularBuffer circularBuffer;
EventSource eventSource;

/* shell command handlers */
static void cmd_reset(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	(void) argc;
	(void) argv;
	chEvtBroadcastFlags(&eventSource, RESET_EVENT_MASK);
}

static void cmd_status(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	(void) argc;
	(void) argv;
	chEvtBroadcastFlags(&eventSource, STATUS_EVENT_MASK);
}

static void cmd_resetled(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	(void) argc;
	(void) argv;
	resetLed();
}

static void cmd_led(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	short offset = 0;
	int x = 0;

	if (argc < 3)
		return;

	switch (argv[0][0])
	{
	case 'R':
		offset = 0;
		break;
	case 'Y':
		offset = 4;
		break;
	case 'G':
		offset = 8;
		break;
	}

	// are we setting the led to the BLINK status?
	if (argv[1][0] == 'B')
	{
		blinking[offset/4] = TRUE;
	}
	else
	{

		blinking[offset/4] = FALSE;
		chMtxLock(&spykeeLedMutex);
		while (argv[1][x] != '\0')
		{
			setLed(offset + x, NUMERIC_CHAR_TO_INT(argv[1][x]));
		}
		chMtxUnlock();
	}
}

static void cmd_infrared(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	if (argc == 1 && argv[0][0] == 'o' && argv[0][1] == 'n')
	{
		palSetPad(IOPORT4, GPIOD_IRLED);
	}
	else
	{
		palClearPad(IOPORT4, GPIOD_IRLED);
	}
}

/* shell configuration */
static const ShellCommand commands[] =
{
{ "reset", cmd_reset },
{ "status", cmd_status },
{ "led", cmd_led },
{ "resetled", cmd_resetled },
{ "infrared", cmd_infrared },
{ NULL, NULL } };

static const ShellConfig shellConfig =
{ (BaseChannel*) &SD2, commands };

/* create and starts the shell (thread waiting for user commands) */
void shellInitControl(Thread** shell)
{
	if (!*shell)
	{
		*shell = shellCreate(&shellConfig, 1024, NORMALPRIO);
	}
	else if (chThdTerminated(*shell))
	{
		*shell = NULL;
	}
}

int main(void)
{
	Thread *shellTp = NULL;
	SerialConfig sd2Config =
	{ SERIAL_OUT_BITRATE, 0, USART_CR2_STOP1_BITS | USART_CR2_LINEN, 0 };

	/* hardware initialization */
	halInit();
	chSysInit();
	sdStart(&SD2, &sd2Config);

	/* Mutex initialization */
	chMtxInit(&bufferMutex);
	chMtxInit(&spykeeLedMutex);

	/* Buffer initialization*/
	bufferInit(&circularBuffer);
	/* event source initialization */
	chEvtInit(&eventSource);

	/* led initialization*/
	resetLed(); /* reset the leds on the robot shoulders */
	palClearPad(IOPORT4, GPIOD_IRLED);
	/* ensure the IR led are turned off */

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
		chMtxLock(&bufferMutex);
		while (!bufferIsEmpty(&circularBuffer))
		{
			chprintf((BaseChannel*) &SD2, "%c", bufferRemove(&circularBuffer));
		}
		chMtxUnlock();
	}
}
