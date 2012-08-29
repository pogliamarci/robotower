#include "FirmwareSpykee.h"

#define CMD_AVOID_UNUSEDVARS_WARNINGS() { (void) channel;(void) argc; (void) argv; }

/* shell command handlers */
static void cmd_reset(BaseChannel* channel, int argc, char** argv)
{
	CMD_AVOID_UNUSEDVARS_WARNINGS();
	chEvtBroadcastFlags(&eventSource, RESET_EVENT_MASK);
}

static void cmd_status(BaseChannel* channel, int argc, char** argv)
{
	CMD_AVOID_UNUSEDVARS_WARNINGS();
	chEvtBroadcastFlags(&eventSource, STATUS_EVENT_MASK);
}

static void cmd_resetled(BaseChannel* channel, int argc, char** argv)
{
	CMD_AVOID_UNUSEDVARS_WARNINGS();
	resetLed();
}

static void cmd_led(BaseChannel* channel, int argc, char** argv)
{
	(void) channel;
	short offset = 0;
	int x = 0;

	if (argc < 2) return;

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
		for(x = 0; argv[1][x] != '\0'; x++)
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
		palSetPad(IOPORT4, GPIOD_IRLED);
	else palClearPad(IOPORT4, GPIOD_IRLED);
}

/* shell configuration */
static const ShellCommand commands[] =
{
	{ "reset", cmd_reset },
	{ "status", cmd_status },
	{ "led", cmd_led },
	{ "resetled", cmd_resetled },
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
