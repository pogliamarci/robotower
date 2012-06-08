#include <string.h>
#include "ch.h"
#include "hal.h"
#include "shell.h"

#define FACTORY_NUMBER 3
#define BUFFER_SIZE 20

static WORKING_AREA(apricancelliWorkingArea, 128);
static WORKING_AREA(shellWorkingArea, 128);

static eventmask_t maskApricancelliEvents = 0x03;

static void cmd_reset(BaseChannel* channel, int argc, char** argv) {
	chprintf(channel, "comando cmd_reset");
}

static void cmd_status(BaseChannel* channel, int argc, char** argv) {
	chprintf(channel, "comando cmd_status");
}

static const ShellCommand commands[] = { { "reset", cmd_reset }, { "status",
		cmd_status }, { NULL, NULL } };

static const ShellConfig shellConfig = { (BaseChannel*) &SD2, commands };

static msg_t apricancelliThread(void *arg) {

	int i;
	int cicla = 1;

	int num_factory = FACTORY_NUMBER;
	int num_tower = 1;
	int destroyed[FACTORY_NUMBER];
	int destroyedT = 0;

	char command_buffer[BUFFER_SIZE];
	char status_buffer[BUFFER_SIZE];

	char* hello =
			"Firmware per leggere le uscite di un apricancelli. Davide Tateo, 2012\n\r";
	char* factoryMessage = "Factory Destroyed\n\r";
	char* towerMessage = "Tower Destroyed\n\r";
	char* evento1 = "Bam giu nel canestro evento1\n\r";
	char* evento2 = "Frocio chi legge l'evento2\n\r";

	eventmask_t eventiArrivati;
	if (arg != NULL)
		i = 0; //TODO: eliminare inutilità a caso
	/*start seriale 2*/
	/* initialize buffers */
	for (i = 0; i < BUFFER_SIZE; i++) {
		command_buffer[i] = '\0';
		status_buffer[i] = '\0';
	}

	for (i = 0; i < FACTORY_NUMBER; i++) {
		destroyed[i] = FALSE;
	}

	/* My welcome message */
	sdWrite(&SD2, (uint8_t *) hello, strlen(hello));

	/* loop apricancelli */
	while (cicla) {
		/* control if a factory has been destroyed */
		for (i = 0; i < FACTORY_NUMBER; i++) {
			if ((palReadPad(IOPORT4, i) == 0) && !destroyed[i]) {
				sdWrite(&SD2, (uint8_t *) factoryMessage,
						strlen(factoryMessage));
				if (num_factory > 0) {
					destroyed[i] = TRUE;
					num_factory--;
				}
			}
		}

		/* control if the tower has been destroyed */
		if ((palReadPad(IOPORT4, 3) == 0) && !destroyedT) {
			sdWrite(&SD2, (uint8_t *) towerMessage, strlen(towerMessage));
			if (num_tower > 0) {
				destroyedT = TRUE;
				num_tower--;
			}
		}

		eventiArrivati = chEvtWaitAnyTimeout(maskApricancelliEvents, 200);
		if (eventiArrivati & 0x01) {
			sdWrite(&SD2, (uint8_t *) evento1, strlen(evento1));

		}

		if (eventiArrivati & 0x02) {
			sdWrite(&SD2, (uint8_t *) evento2, strlen(evento2));
		}

		chEvtClearFlags(maskApricancelliEvents);
	}
	return 0;
}

int main(void) {

	Thread *shelltp = NULL;
	Thread *tp;
	EventSource eventSource;
	EventListener eventListener;
	halInit();
	chSysInit();

	sdStart(&SD2, NULL);

	chEvtInit(&eventSource);
	chEvtRegisterMask(&eventSource, &eventListener, maskApricancelliEvents);

	tp = chThdCreateStatic(apricancelliWorkingArea,
	 sizeof(apricancelliWorkingArea), NORMALPRIO, apricancelliThread,
	 NULL);

	 chEvtSignalFlags(tp, maskApricancelliEvents);

	shellInit();

	while (TRUE) {
		if (!shelltp) {
			shelltp = shellCreate(&shellConfig, 1024, NORMALPRIO);
		} else if (chThdTerminated(shelltp)) {
			shelltp = NULL;
		}
		chEvtBroadcastFlags(&eventSource, 0x01);
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
		chEvtBroadcastFlags(&eventSource, 0x02);
		chThdSleepMilliseconds(500);
		chEvtSignalFlags(tp, maskApricancelliEvents);

	}
}
