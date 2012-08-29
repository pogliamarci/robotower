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

static WORKING_AREA(TowersAndFactoriesWorkingArea, 256);

/* enable the blinking mode for the three groups of leds on the robot
 * -> blinking[0] controls the red leds
 * -> blinking[1] controls the yellow leds,
 * -> blinking[2] controls the green led */
bool_t blinking[] = {FALSE, FALSE, FALSE};

void writeStatusToBuffer(int num_factory, int tower_destroyed)
{
	char statusMessage[20];
	chsprintf(statusMessage, "[TOWER] F:%d,T:%d", num_factory,
			tower_destroyed ? 0 : 1);
	chMtxLock(&bufferMutex);
	bufferPutString(&circularBuffer, statusMessage);
	chMtxUnlock();
}

/* thread that manages gate opener signals */
static msg_t towerFactoriesThread(void *arg)
{
	(void) arg;
	int i;
	int num_factory = FACTORY_NUMBER;
	int destroyed[FACTORY_NUMBER];
	int tower_destroyed = 0;

	EventListener eventListener;
	eventmask_t eventiArrivati;

	/* initialize the event listener */
	chEvtRegisterMask(&eventSource, &eventListener, 0);

	for (i = 0; i < FACTORY_NUMBER; i++)
	{
		destroyed[i] = FALSE;
	}

	/* loop apricancelli */
	while (TRUE)
	{
		/* control if a factory has been destroyed */
		for (i = 0; i < FACTORY_NUMBER; i++)
		{
			if ((palReadPad(IOPORT4, i) == 0) && !destroyed[i])
			{
				destroyed[i] = TRUE;
				num_factory--;
				writeStatusToBuffer(num_factory, tower_destroyed);
			}
		}

		/* control if the tower has been destroyed */
		if ((palReadPad(IOPORT4, 3) == 0) && !tower_destroyed)
		{
			tower_destroyed = TRUE;
			writeStatusToBuffer(num_factory, tower_destroyed);
		}

		/* event management, for now timeout has been set to 500 ms */
		eventiArrivati = chEvtWaitAnyTimeout(ALL_EVENTS, 500);
		if (eventiArrivati & RESET_EVENT_MASK)
		{ /* reset event */
			for (i = 0; i < FACTORY_NUMBER; i++)
			{
				destroyed[i] = FALSE;
			}
			tower_destroyed = FALSE;
			num_factory = FACTORY_NUMBER;
		}
		if (eventiArrivati & STATUS_EVENT_MASK)
		{ /* status event */
			writeStatusToBuffer(num_factory, tower_destroyed);
		}
	}
	return 0;
}

void startTowersAndFactoriesThread(void)
{
	chThdCreateStatic(TowersAndFactoriesWorkingArea, sizeof(TowersAndFactoriesWorkingArea),
				NORMALPRIO, towerFactoriesThread, NULL );
}




