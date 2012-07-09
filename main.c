#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"

#define FACTORY_NUMBER 3

#define RESET_EVENT_MASK 0x01
#define STATUS_EVENT_MASK 0x02

static WORKING_AREA(apricancelliWorkingArea, 256);
EventSource eventSource;

/* shell command handlers */
static void cmd_reset(BaseChannel* channel, int argc, char** argv)
{
	chprintf(channel, "reset\n\r");
	chEvtBroadcastFlags(&eventSource, RESET_EVENT_MASK);
}

static void cmd_status(BaseChannel* channel, int argc, char** argv)
{
	chprintf(channel, "status\n\r");
	chEvtBroadcastFlags(&eventSource, STATUS_EVENT_MASK);
}

/* shell configuration */
static const ShellCommand commands[] = {
		{ "reset", cmd_reset },
		{ "status", cmd_status },
		{ NULL, NULL }
};

static const ShellConfig shellConfig = {
		(BaseChannel*) &SD2, commands
};

static msg_t apricancelliThread(void *arg)
{
	int i;
	int num_factory = FACTORY_NUMBER;
	int destroyed[FACTORY_NUMBER];
	int tower_destroyed = 0;

	char* factoryMessage = "Factory Destroyed\n\r";
	char* towerMessage = "Tower Destroyed\n\r";

	EventListener eventListener;
	eventmask_t eventiArrivati;

	/* initialize the event listener */
	chEvtRegisterMask(&eventSource, &eventListener, 0);

	for (i = 0; i < FACTORY_NUMBER; i++) {
		destroyed[i] = FALSE;
	}

	/* loop apricancelli */
	while (TRUE) {
		/* control if a factory has been destroyed */
		for (i = 0; i < FACTORY_NUMBER; i++) {
			if ((palReadPad(IOPORT4, i) == 0) && !destroyed[i]) {
				sdWrite(&SD2, (uint8_t *) factoryMessage, strlen(factoryMessage));
				destroyed[i] = TRUE;
				num_factory--;
			}
		}

		/* control if the tower has been destroyed */
		if ((palReadPad(IOPORT4, 3) == 0) && !tower_destroyed) {
			sdWrite(&SD2, (uint8_t *) towerMessage, strlen(towerMessage));
			tower_destroyed = TRUE;
		}

		/* event management, for now timeout has been set to 500 ms */
		eventiArrivati = chEvtWaitAnyTimeout(ALL_EVENTS, 500);
		if (eventiArrivati & RESET_EVENT_MASK) { /* reset event */
			for (i = 0; i < FACTORY_NUMBER; i++) {
				destroyed[i] = FALSE;
			}
			tower_destroyed = FALSE;
			num_factory = FACTORY_NUMBER;
		}
		if (eventiArrivati & STATUS_EVENT_MASK) { /* status event */
			chprintf(&SD2, "F:%d,T:%d\r\n", num_factory,
					tower_destroyed ? 0 : 1);
		}
	}
	return 0;
}

/* blink sequentially the four integrated leds of the STM32F4Discovery */
void blinkLeds(void)
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

//TODO: rfid, scrittura su zigbee (banali, ma da fare)
//TODO: led fatti per bene? magari poi...
int main(void)
{
	Thread *shelltp = NULL;
	Thread *tp;

	/* hardware initialization */
	halInit();
	chSysInit();
	sdStart(&SD2, NULL);

	/* event source initialization */
	chEvtInit(&eventSource);

	/* thread initialization */
	tp = chThdCreateStatic(apricancelliWorkingArea,
			sizeof(apricancelliWorkingArea), NORMALPRIO, apricancelliThread,
			NULL);
	shellInit();

	while (TRUE)
	{
		/* create and starts the shell (thread waiting for user commands) */
		if (!shelltp) {
			shelltp = shellCreate(&shellConfig, 1024, NORMALPRIO);
		} else if (chThdTerminated(shelltp)) {
			shelltp = NULL;
		}
		blinkLeds();
	}
}
