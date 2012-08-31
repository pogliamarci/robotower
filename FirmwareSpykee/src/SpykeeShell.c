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

#define NUMERIC_CHAR_TO_INT(c) ((char) (c) - '0')

/* Turns off the leds on the robot shoulders and the infrared led */
static void cmd_reset(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	(void) argc;
	(void) argv;
	resetLed();
}

/* Send a command to the leds on the robot shoulders. */
static void cmd_led(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	short ledgroup = 0;
	// short offset = 0;
	int x = 0;

	if (argc < 2) return;

	switch (argv[0][0])
	{
	case 'R':
		ledgroup = 0;
		break;
	case 'Y':
		ledgroup = 1;
		break;
	case 'G':
		ledgroup = 2;
		break;
	}

	// are we setting the led to the BLINK status?
	if (argv[1][0] == 'B')
	{
		blinking[ledgroup] = TRUE;
	}
	else
	{
		blinking[ledgroup] = FALSE;
		chMtxLock(&spykeeLedMutex);
		for(x = 0; argv[1][x] != '\0'; x++)
		{
			setLed(ledgroup * 4 + x, NUMERIC_CHAR_TO_INT(argv[1][x]));
		}
		chMtxUnlock();
	}
}

/* send a command ('on' or 'off') to the infrared leds */
static void cmd_infrared(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	if (argc == 1 && argv[0][0] == 'o' && argv[0][1] == 'n')
		palSetPad(IOPORT4, GPIOD_IRLED);
	else palClearPad(IOPORT4, GPIOD_IRLED);
}

static const ShellCommand commands[] =
{
	{ "reset", cmd_reset },
	{ "led", cmd_led },
	{ "infrared", cmd_infrared },
	{ NULL, NULL }
};

static const ShellConfig shellConfig = { (BaseChannel*) &SD2, commands };

/* create and starts the shell (thread waiting for user commands) */
void shellInitControl(Thread** shell)
{
	if (!*shell)
		*shell = shellCreate(&shellConfig, 1024, NORMALPRIO);
	else if (chThdTerminated(*shell))
		*shell = NULL;
}
