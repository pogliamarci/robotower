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

#ifndef FIRMWARESPYKEE_H_
#define FIRMWARESPYKEE_H_

#include "ch.h"
#include "hal.h"
#include "shell.h"

#include "chsprintf.h"
#include "CircularBuffer.h"

#define NUM_LED_GROUPS 3

/* Functions */
void startLedBlinkerTreads(void);
void setLed(int n, bool_t setOn);
void resetLed(void);

void startTowersAndFactoriesThread(void);
void startRfidThread(void);
void startSonarThread(void);
void shellInitControl(Thread** shell);

/* Set the mode of a group of leds to 'blinking'.
 * Indexes are 0 for, 1 for and 2 for */
extern bool_t blinking[NUM_LED_GROUPS];

/* used to synchronize read\writes to the leds (gpios),
 * to avoid race conditions with the blinker thread */
extern Mutex spykeeLedMutex;

/* The buffer used to write on the serial port connected to the PC */
extern CircularBuffer outputBuffer;

#endif /* FIRMWARESPYKEE_H_ */
