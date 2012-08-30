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

#ifndef FIRMWARESPIKEE_H_
#define FIRMWARESPYKEE_H_


#include "ch.h"
#include "hal.h"
#include "shell.h"

#include "chsprintf.h"
#include "CircularBuffer.h"

#define SERIAL_OUT_BITRATE 19200

#define NUMERIC_CHAR_TO_INT(c) ((char) (c) - '0')

/* events declaration */
#define RESET_EVENT_MASK 0x01
#define STATUS_EVENT_MASK 0x02

/* Functions */
void startLedTreads(void);
void setLed(int n, bool_t setOn);
void resetLed(void);

void startTowersAndFactoriesThread(void);
void startRfidThread(void);
void startSonarThread(void);
void shellInitControl(Thread** shell);

/* Global Variables */
extern bool_t blinking[];
extern Mutex spykeeLedMutex;
extern CircularBuffer outputBuffer;

#endif /* FIRMWARESPYKEE_H_ */
