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

#define REFRESH_INTERVAL 100 // milliseconds
#define TOWER_NUMBER 4

static WORKING_AREA(TowersAndFactoriesWorkingArea, 256);

void writeTowerEventToBuffer(int tower)
{
	char statusMessage[20];
	chsprintf(statusMessage, "[TOWER] destroyed %d", tower+1);
	bufferPutString(&outputBuffer, statusMessage);
}

/* thread that manages gate opener signals */
static msg_t towerFactoriesThread(void *arg)
{
	(void) arg;

	int i;
	bool_t destroyed[TOWER_NUMBER];
	bool_t curIsDestroyed;

	for (i = 0; i < TOWER_NUMBER; i++)
	{
		destroyed[i] = FALSE;
	}

	while (TRUE)
	{
		for (i = 0; i < TOWER_NUMBER; i++)
		{
			// towers are from PD0 to PD4
			curIsDestroyed = (palReadPad(IOPORT4, i) == 0);
			if (curIsDestroyed && !destroyed[i])
			{
				destroyed[i] = TRUE;
				writeTowerEventToBuffer(i);
			}
			else if (!curIsDestroyed && destroyed[i])
			{
				destroyed[i] = FALSE;
			}
		}
		chThdSleepMilliseconds(REFRESH_INTERVAL);
	}
	return 0;
}

void startTowersAndFactoriesThread(void)
{
	chThdCreateStatic(TowersAndFactoriesWorkingArea,
			sizeof(TowersAndFactoriesWorkingArea), NORMALPRIO,
			towerFactoriesThread, NULL );
}
