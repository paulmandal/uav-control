/* js_ctrl.c
 *
 * Tracks joystick updates
 * Sends JS state every 20ms
 * Receives commands from PPZ, relays to UAV
 * Receives telemetry from UAV, relays to PPZ
 *
 * Special thanks to Vojtech Pavlik <vojtech@ucw.cz>, I adopted much of this code from jstest.c
*/

/*

TODO:

- Interpret JS inputs (e.g. toggles, button presses, contexts, etc)
- Generate JS state command every pulse
- Relay PPZ -> UAV
- Relay UAV -> PPZ

Adruino:

- Read + Interpret JS state command
- Set in-memory JS model
- Set digital pins
- Generate PPM

*/



#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "axbtnmap.h"

char *axis_names[ABS_MAX + 1] = {
"X", "Y", "Z", "Rx", "Ry", "Rz", "Throttle", "Rudder", 
"Wheel", "Gas", "Brake", "?", "?", "?", "?", "?",
"Hat0X", "Hat0Y", "Hat1X", "Hat1Y", "Hat2X", "Hat2Y", "Hat3X", "Hat3Y",
"?", "?", "?", "?", "?", "?", "?", 
};

char *button_names[KEY_MAX - BTN_MISC + 1] = {
"Btn0", "Btn1", "Btn2", "Btn3", "Btn4", "Btn5", "Btn6", "Btn7", "Btn8", "Btn9", "?", "?", "?", "?", "?", "?",
"LeftBtn", "RightBtn", "MiddleBtn", "SideBtn", "ExtraBtn", "ForwardBtn", "BackBtn", "TaskBtn", "?", "?", "?", "?", "?", "?", "?", "?",
"Trigger", "ThumbBtn", "ThumbBtn2", "TopBtn", "TopBtn2", "PinkieBtn", "BaseBtn", "BaseBtn2", "BaseBtn3", "BaseBtn4", "BaseBtn5", "BaseBtn6", "BtnDead",
"BtnA", "BtnB", "BtnC", "BtnX", "BtnY", "BtnZ", "BtnTL", "BtnTR", "BtnTL2", "BtnTR2", "BtnSelect", "BtnStart", "BtnMode", "BtnThumbL", "BtnThumbR", "?",
"?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", "?", 
"WheelBtn", "Gear up",
};

#define NAME_LENGTH 128
#define PPM_INTERVAL 20000

struct js_state {
		int *axis;
		char *button;	
};

struct js_state js_st;

/* Timer stuff */

void send_ctrl_update (int signum)
{
	printf("\r");
	
	if (axes) {
		printf("Axes: ");
		for (i = 0; i < axes; i++)
			printf("%2d:%6d ", i, axis[i]);
	}

	if (buttons) {
		printf("Buttons: ");
		for (i = 0; i < buttons; i++)
			printf("%2d:%s ", i, button[i] ? "on " : "off");
	}

	fflush(stdout);

	// generate and send output string

	// possible:

	// pulse but no change: CMDXEND
	// changes: S[##][DEG] for each servo
	// P[##][0/1] for each digital pin

}

int main (int argc, char **argv)
{
	int fd, i;
	unsigned char axes = 2;
	unsigned char buttons = 2;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";
	uint16_t btnmap[BTNMAP_SIZE];
	uint8_t axmap[AXMAP_SIZE];
	int btnmapok = 1;

	// timer stuff

	struct sigaction sa;
	struct itimerval timer;

	if (argc != 2 || !strcmp("--help", argv[1])) {
		puts("");
		puts("Usage: js_ctrl <XBee UART> <PPZ UART> <joystick device>");
		puts("");
		return 1;
	}

	printf("Opening joystick..\n");
	if ((fd = open(argv[argc - 1], O_RDONLY | O_NONBLOCK)) < 0) {
		perror("js_ctrl");
		return 1;
	}

	ioctl(fd, JSIOCGVERSION, &version);
	ioctl(fd, JSIOCGAXES, &axes);
	ioctl(fd, JSIOCGBUTTONS, &buttons);
	ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);

	getaxmap(fd, axmap);
	getbtnmap(fd, btnmap);

	printf("Driver version is %d.%d.%d.\n",
		version >> 16, (version >> 8) & 0xff, version & 0xff);

	/* Determine whether the button map is usable. */
	for (i = 0; btnmapok && i < buttons; i++) {
		if (btnmap[i] < BTN_MISC || btnmap[i] > KEY_MAX) {
			btnmapok = 0;
			break;
		}
	}
	if (!btnmapok) {
		/* btnmap out of range for names. Don't print any. */
		puts("js_ctrl is not fully compatible with your kernel. Unable to retrieve button map!");
		printf("Joystick (%s) has %d axes ", name, axes);
		printf("and %d buttons.\n", buttons);
	} else {
		printf("Joystick (%s) initialised with %d axes and %d buttons.\n", name, axes, buttons);
	}


	printf("Ready to read JS & relay for PPZ...\n");

	int i;
	int ticker = 0;
	struct js_event js;

	js_st->axis = calloc(axes, sizeof(int));
	js_st->button = calloc(buttons, sizeof(char));

	printf("Setting up timer..\n");
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &send_ctrl_update;
	sigaction (SIGALRM, &sa, NULL);

	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = PPM_INTERVAL;
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = PPM_INTERVAL;
	setitimer (ITIMER_REAL, &timer, NULL);

	while (1) {
		if (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				js_st->button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				js_st->axis[js.number] = js.value;
				break;
			}

		}

		usleep(1);
		ticker++;

		if(ticker > 100000) {

			ticker = 0;
			printf("ticked!\r");
	
			if (axes) {
				printf("Axes: ");
				for (i = 0; i < axes; i++)
					printf("%2d:%6d ", i, axis[i]);
			}

			if (buttons) {
				printf("Buttons: ");
				for (i = 0; i < buttons; i++)
					printf("%2d:%s ", i, button[i] ? "on " : "off");
			}

			fflush(stdout);


		}
	}

	return 0;

}
