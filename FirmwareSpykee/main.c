#include <string.h>
#include "ch.h"
#include "hal.h"

#define FACTORY_NUMBER 3
#define BUFFER_SIZE 20

static WORKING_AREA(apricancelliWorkingArea, 128);

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
			" Firmware per leggere le uscite di un apricancelli. Davide Tateo, 2012\n\r";
	char* factoryMessage = "Factory Destroyed\n\r";
	char* towerMessage = "Tower Destroyed\n\r";

	if(arg!=NULL) i=0; //TODO: eliminare inutilità a caso
	/*start seriale 2*/
	sdStart(&SD2, NULL);
	/* initialize buffers */
	for (i = 0; i < BUFFER_SIZE; i++) {
		command_buffer[i] = '\0';
		status_buffer[i] = '\0';
	}

	for (i = 0; i < FACTORY_NUMBER; i++) {
		destroyed[i] = FALSE;
	}

	/* My welcome message */
	sdWrite(&SD2, (uint8_t *)hello, strlen(hello));

	/* loop apricancelli */
	while (cicla)
	{
		/* control if a factory has been destroyed */
		for (i = 0; i < FACTORY_NUMBER; i++) {
			if ((palReadPad(IOPORT4,i) == 0) && !destroyed[i]) {
				sdWrite(&SD2, (uint8_t *)factoryMessage,
						strlen(factoryMessage));
				if (num_factory > 0) {
					destroyed[i] = TRUE;
					num_factory--;
				}
			}
		}

		/* control if the tower has been destroyed */
		if ((palReadPad(IOPORT4,3) == 0) && !destroyedT) {
			sdWrite(&SD2, (uint8_t *)towerMessage, strlen(towerMessage));
			if (num_tower > 0) {
				destroyedT = TRUE;
				num_tower--;
			}
		}

		//TODO: sposta nel main
		/* input processing */
		/*if (arrived_char_usart1()) {
		 read_string_usart1(command_buffer, '\r', BUFFER_SIZE); //TODO: togliere schifo per screen

		 if (strcmp(command_buffer, "status") == 0) {
		 sprintf(status_buffer, "#F=%d; #D=%d\n\r", num_factory,
		 num_tower); //TODO: togliere schifo per screen
		 print_string_usart1(status_buffer);
		 } else if (strcmp(command_buffer, "reset") == 0) {
		 num_factory = 3;
		 num_tower = 1;
		 for (i = 0; i < FACTORY_NUMBER; i++)
		 destroyed[i] = FALSE;
		 destroyedT = FALSE;
		 } else if (strcmp(command_buffer, "stop") == 0) {
		 cicla = FALSE;
		 print_string_usart1("Exit the main loop...\n\r"); //TODO: togliere schifo per screen
		 } else {
		 print_string_usart1("Command not found\n\r"); //TODO: togliere schifo per screen
		 }
		 }*/

	}
	return 0;
}

int main(void) {

	char hello[] = "Hello World!\n\r";

	halInit();
	chSysInit();

	/* set up the serial port */
	sdStart(&SD1, NULL);
	//sdStart(&SD2, NULL);
	sdStart(&SD3, NULL);
	sdStart(&SD6, NULL);

	Thread *tp = chThdCreateStatic(apricancelliWorkingArea,
				sizeof(apricancelliWorkingArea), NORMALPRIO, apricancelliThread,
				NULL);

	tp++; //TODO: eliminare inutilità a caso
	while (TRUE) {
		palClearPad(IOPORT4, GPIOD_LED4);
		palClearPad(IOPORT4, GPIOD_LED6);
		chThdSleepMilliseconds(500);
		palSetPad(IOPORT4, GPIOD_LED4);
		palSetPad(IOPORT4, GPIOD_LED6);
		sdWrite(&SD1, (uint8_t *) hello, 14);
		//sdWrite(&SD2, (uint8_t *) hello, 14);
		sdWrite(&SD3, (uint8_t *) hello, 14);
		sdWrite(&SD6, (uint8_t *) hello, 14);
		chThdSleepMilliseconds(500);
	}
}
