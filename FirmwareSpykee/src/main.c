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

#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chsprintf.h"
#include "chprintf.h"
#include "CircularBuffer.h"

#define FACTORY_NUMBER 3
#define SERIAL_OUT_BITRATE 19200

#define NUMERIC_CHAR_TO_INT(c) ((char) (c) - '0')

/* events declaration */
#define RESET_EVENT_MASK 0x01
#define STATUS_EVENT_MASK 0x02

/* working areas declarations for the threads */
static WORKING_AREA(apricancelliWorkingArea, 256);
static WORKING_AREA(sonarWorkingArea, 1024);
static WORKING_AREA(rfidWorkingArea, 1024);
static WORKING_AREA(blinkerWorkingArea, 128);
static WORKING_AREA(spykeeLedBlinkerWorkingArea, 128);

CircularBuffer circularBuffer;
Mutex bufferMutex;
Mutex spykeeLedMutex;
EventSource eventSource;

/* enable the blinking mode for the three groups of leds on the robot
 * -> blinking[0] controls the red leds
 * -> blinking[1] controls the yellow leds,
 * -> blinking[2] controls the green led */
bool_t blinking[] = {FALSE, FALSE, FALSE};

typedef struct
{
	icucnt_t north;
	icucnt_t south;
	icucnt_t west;
	icucnt_t east;
} SonarData;

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

// led {Y, R} {mask, blink}
// led G {0,1,blink}

static const ShellConfig shellConfig =
{ (BaseChannel*) &SD2, commands };

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
		if(blinking[0])
		{
			setLed(redStat, FALSE);
			redStat = redStat == 3 ? 1 : redStat + 1;
			setLed(redStat, TRUE);
		}
		if(blinking[1])
		{
			setLed(4 + yellowStat, FALSE);
			yellowStat = yellowStat == 3 ? 1 : yellowStat + 1;
			setLed(yellowStat, TRUE);
		}
		if(blinking[2])
		{
			setLed(8, greenStat);
			greenStat = !greenStat;
		}
		chMtxUnlock();
		chThdSleepMilliseconds(500);
	}
	return 0;
}

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
		chMtxLock(&bufferMutex);
		bufferPutString(&circularBuffer, buf);
		chMtxUnlock();
		chThdSleepMilliseconds(100);
	}

	return 0;
}

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

		chMtxLock(&bufferMutex);
		bufferPutString(&circularBuffer, buf2);
		chMtxUnlock();
	}
	return 0;
}

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
	chThdCreateStatic(apricancelliWorkingArea, sizeof(apricancelliWorkingArea),
			NORMALPRIO, towerFactoriesThread, NULL );
	chThdCreateStatic(blinkerWorkingArea, sizeof(blinkerWorkingArea),
			NORMALPRIO, blinkerThread, NULL );
	chThdCreateStatic(sonarWorkingArea, sizeof(sonarWorkingArea), NORMALPRIO,
			sonarThread, NULL );
	chThdCreateStatic(rfidWorkingArea, sizeof(rfidWorkingArea), NORMALPRIO,
			rfidThread, NULL );
	chThdCreateStatic(spykeeLedBlinkerWorkingArea, sizeof(spykeeLedBlinkerWorkingArea),
			NORMALPRIO, spykeeLedBlinkerThread, NULL);
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
