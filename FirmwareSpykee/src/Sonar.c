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

typedef struct
{
	icucnt_t north;
	icucnt_t south;
	icucnt_t west;
	icucnt_t east;
} SonarData;

static WORKING_AREA(sonarWorkingArea, 1024);

/* ICU callbacks and data to be used with sonars */

SonarData sonar_data =
{ 0, 0, 0, 0 };

static void icuWidthCB(ICUDriver *icup)
{
	/* compute the measured distance */
	const int scaleFactor = 5787; //5,7874 us/mm
	icucnt_t width = icuGetWidthI(icup) * 1000 / scaleFactor;
	/* set the right variable */
	if (icup == &ICUD1)
		sonar_data.north = width;
	else if (icup == &ICUD3)
		sonar_data.south = width;
	else if (icup == &ICUD5)
		sonar_data.west = width;
	else if (icup == &ICUD8)
		sonar_data.east = width;
}

/* Thread used for read data from sonar */
static msg_t sonarThread(void *arg)
{
	(void) arg;
	char buf[40];
	const int frequenzaTimer = 1000000; //T = 1uS
	ICUConfig icucfg =
	{ ICU_INPUT_ACTIVE_HIGH, frequenzaTimer, icuWidthCB, NULL };
	icuStart(&ICUD1, &icucfg); //PA8, ICDU1
	icuStart(&ICUD3, &icucfg); //PB4, ICDU3
	icuStart(&ICUD5, &icucfg); //PA0, ICDU5
	icuStart(&ICUD8, &icucfg); //PC6, ICDU8
	/* icu enabling */
	icuEnable(&ICUD1);
	icuEnable(&ICUD3);
	icuEnable(&ICUD5);
	icuEnable(&ICUD8);
	while (TRUE)
	{
		chsprintf(buf, "[SONAR] N:%d,S:%d,W:%d,E:%d", sonar_data.north,
				sonar_data.south, sonar_data.west, sonar_data.east);
		/* let's "consume" the data in the buffer... */
		sonar_data.north = 0;
		sonar_data.south = 0;
		sonar_data.west = 0;
		sonar_data.east = 0;
		bufferPutString(&outputBuffer, buf);
		chThdSleepMilliseconds(100);
	}

	return 0;
}

void startSonarThread(void)
{
	chThdCreateStatic(sonarWorkingArea, sizeof(sonarWorkingArea), NORMALPRIO,
			sonarThread, NULL );
}

