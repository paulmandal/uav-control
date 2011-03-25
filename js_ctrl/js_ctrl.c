/* js_ctrl.c
 *
 * Tracks joystick updates
 * Sends JS state every 20ms
 * Receives commands from PPZ, relays to UAV
 * Receives telemetry from UAV, relays to PPZ
 *
 * Special thanks to Vojtech Pavlik <vojtech@ucw.cz>, I adopted much of the joystick code from jstest.c
*/

/*

TODO:

- DPad -> servo control
- Generate JS state command every pulse
- Relay PPZ -> UAV
- Relay UAV -> PPZ

Adruino:

- Read + Interpret JS state command
- Set in-memory JS model
- Set digital pins
- Generate PPM

*/

// include necessary libs

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <termios.h> /* POSIX terminal control definitions */

#include <linux/input.h>
#include <linux/joystick.h>

#include "axbtnmap.h"

#include <signal.h>

// definitions

#define NAME_LENGTH 128
#define PPM_INTERVAL 100000
#define JS_DISCARD_UNDER 5000
#define CMD_PREFIX 0xFF
#define MSG_SIZE 12
#define CAM_PAN 4
#define CAM_TILT 5
#define ROLL 0
#define PITCH 1
#define YAW 3
#define THROTTLE 2

#define SERVO_COUNT 8
#define BUTTON_COUNT 12

#define SRV_L_MAX 180
#define SRV_L_MIN 0

#define SRV_R_MAX 180
#define SRV_R_MIN 0

#define SRV_L_AL_MAX 180
#define SRV_L_AL_MIN 0

#define SRV_R_AL_MAX 180
#define SRV_R_AL_MIN 0

#define ESC_R_MAX 255
#define ESC_R_MIN 0

#define ESC_L_MAX 255
#define ESC_L_MIN 0

#define CAM_TILT_MAX 180
#define CAM_TILT_MIN 0

#define CAM_PAN_MAX 180
#define CAM_PAN_MIN 0

#define PPZ_MSG_SIZE 1024

#define SRV_ESC_LEFT 0
#define SRV_ESC_RIGHT 1
#define SRV_LEFTWING 2
#define SRV_RIGHTWING 3
#define SRV_L_ELEVRON 4
#define SRV_R_ELEVRON 5
#define SRV_CAM_PAN 6
#define SRV_CAM_TILT 7

// structures

struct js_state {
		int *axis;
		char *button;	
};

struct airframe_state {

	unsigned int servos[8];
	unsigned int buttons[12];

};

// let's do sum prototypes!

int mapRange(int a1,int a2,int b1,int b2,int s);
void sendCtrlUpdate (int signum);
void readJoystick(int js_port, struct js_state js_st);
int openPort(char *portName, char *use);
int openJoystick(char *portName);
int doHandshake(int xbee_port);
void setupTimer();
void translateJStoAF(struct js_state js_st);

// global config vars

int toggle_buttons[12] = {
0, 1, 0, 1, 
0, 1, 0, 1, 
0, 1, 0, 1
};

int button_state_count[12] = {
2, 0, 0, 0, 
0, 0, 0, 0, 
0, 0, 0, 0
};

double dpad_axis[2] = {
0.0, 0.0
};

int dpad_state[2] = {
0, 0
};

// global variables to store states

struct airframe_state af_st;
unsigned char axes = 2;
unsigned char buttons = 2;
int ppz_port;
int xbee_port;

// main

int main (int argc, char **argv)
{
	int i, js_port, js_value, msg_wrote, old_throttle = 0;
	struct js_state js_st;
	char xbee_buffer;
	char ppz_buffer;

	// init airframe

	for(i = 0 ; i < 8 ; i++) {

		af_st.servos[i] = 0;

	}
	
	if (argc < 4 || !strcmp("--help", argv[1])) {
		puts("");
		puts("Usage: js_ctrl <XBee UART> <PPZ UART> <joystick device>");
		puts("");
		return 1;
	}

	// open the XBee port

	if((xbee_port = openPort(argv[argc - 3], "XBee")) < 0) {
		return 1;
	}

	// open the PPZ port

	/*if((ppz_port = openPort(argv[argc - 2], "PPZ")) < 0) {
		return 1;
	}*/

	// open the Joystick

	if((js_port = openJoystick(argv[argc - 1])) < 0) {
		return 1;
	}

	// do handshaking w/ Arduino

	if(doHandshake(xbee_port) < 0) {
		return 1;
	}

	// set up timer

	setupTimer();

	printf("Ready to read JS & relay for PPZ...\n");

	js_st.axis = calloc(axes, sizeof(int));
	js_st.button = calloc(buttons, sizeof(char));

	while (1) {

		// check joystick

		readJoystick(js_port, &js_st);

		// update Airframe model
		
		translateJStoAF(js_st);
		
		// check XBee port

		while(read(xbee_port, &xbee_buffer, 1) > 0) {

			printf("%s", xbee_buffer);

		}
		printf("\n");

		// check PPZ port

		/*while(read(ppz_port, &ppz_buffer, 1) > 0) {

			if(first == 1) {

				printf("Read from PPZ: %s", ppz_buffer);
				first = 0;

			} else {

				printf("%s", ppz_buffer);
	
			}

		}*/

		usleep(1);
	}

	return 0;

}

// function definitions

// mapRange() - map a number in a1->a2 range into b1->b2
int mapRange(int a1,int a2,int b1,int b2,int s)
{
	return b1 + (s-a1)*(b2-b1)/(a2-a1);
}

// openPort() - open a UART
int openPort(char *portName, char *use) {

	int fd;
	printf("Opening serial port %s for %s..\n", portName, use);

	if ((fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("js_ctrl");
		return -1;
	}

	struct termios options;

	tcgetattr(fd, &options);
	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);
	options.c_cflag |= (CLOCAL | CREAD);
	tcsetattr(fd, TCSANOW, &options);

	return fd;

}

// openJoystick() - get the joystick

int openJoystick(char *portName) {

	uint16_t btnmap[BTNMAP_SIZE];
	uint8_t axmap[AXMAP_SIZE];
	int btnmapok = 1;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";

	printf("Opening joystick %s..\n", portName);
	if ((fd = open(portName, O_RDONLY | O_NONBLOCK)) < 0) {
		perror("js_ctrl");
		return -1;
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

	return fd;

}

// doHandshake() - handshake with Arduino

int doHandshake(int xbee_port) {

	char handshake_msg[MSG_SIZE];
	char handshake_ack[3];
	bool handshook = false;

	for(i = 0 ; i < MSG_SIZE ; i++) {

		handshake_msg[i] = CMD_PREFIX;

	}	

	printf("Handshaking...");

	while(!handshook) {

		msg_wrote = write(xbee_port, handshake_msg, MSG_SIZE);
		if(msg_wrote != MSG_SIZE) {

			printf("Handshaking: error writing to XBee, wrote: %d, msg_size: %d\n", msg_wrote, MSG_SIZE);

		}

		sleep(20); // 20ms to respond
		
		if(read(xbee_port, &handshake_ack, 3) == 3) {

			if(!strcmp(handshake_ack, "ACK")) {

				handshook = true;

			}

		}


	}

	printf("...got ACK, handshake complete!\n");

	return 1;	

}

// setupTimer() - set up pulse timer
void setupTimer() {

	struct sigaction sa;
	struct itimerval timer;
	
	printf("Setting up timer..\n");
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &sendCtrlUpdate;
	sigaction (SIGALRM, &sa, NULL);

	timer.it_value.tv_sec = 1;
	timer.it_value.tv_usec = 0;
	timer.it_interval.tv_sec = 1;
	timer.it_interval.tv_usec = 0;

	printf("Starting pulse timer..\n");
	setitimer (ITIMER_REAL, &timer, NULL);

}

void readJoystick(int js_port, struct js_state js_st) {

	struct js_event js;

	while(read(js_port, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:

				// check if the button is a toggle or not
				if(toggle_buttons[js.number] == 1) {

					if(js.value == 1) {

						js_st.button[js.number] = !js_st.button[js.number];

					}

				// check if the button has multiple states, if so, cycle through them				

				} else if(button_state_count[js.number] > 0) {

					if(js.value == 1) {

						if(js_st.button[js.number] < button_state_count[js.number]) {

							js_st.button[js.number] = js_st.button[js.number] + 1;	

						} else {

							js_st.button[js.number] = 0;

						}

					}

				// normal button, switch to value received by event

				} else {

					js_st.button[js.number] = js.value;

				}


				break;
			case JS_EVENT_AXIS:
				
				// disregard small values (probably stuck sticks)

				if(abs(js.value) > JS_DISCARD_UNDER) {
			
					js_value = js.value;
					
				} else {

					js_value = 0;

				}

				// handle throttle and other axes

				if(js.number == THROTTLE) {

					// invert throttle
					js_value = js_value * -1;

					if(js_value > 0) {

						if(js_value > old_throttle) {

							js_st.axis[js.number] = js_value;
							old_throttle = js_value;

						}

					} else {

						// we need to map this number (-32767->0 to 0->32767)
						js_value = js_value + 32767;
						if(js_value < old_throttle) {

							js_st.axis[js.number] = js_value;
							old_throttle = js_value;

						}

					}
					
				} else {

					js_st.axis[js.number] = js_value;

				}
				break;

			}

		}

}

void translateJStoAF(struct js_state js_st) {

	int x;
	if(js_st.axis[ROLL] > 0) {

		if(js_st.axis[ROLL] < (32767 / 2)) {

			// less than halfway to maximum 
			// only adjust lefthand servo
			
			af_st.servos[SRV_LEFTWING] = mapRange(0, 32767, SRV_L_MIN, SRV_L_MAX, js_st.axis[ROLL]);

		} else {
			
			af_st.servos[SRV_LEFTWING] = mapRange(0, 32767, SRV_L_MIN, SRV_L_MAX, js_st.axis[ROLL]);
			af_st.servos[SRV_RIGHTWING] = mapRange(0, 32767, SRV_R_MAX / 2, SRV_R_MIN / 2, js_st.axis[ROLL]);

		}

	} else {

		if(js_st.axis[ROLL] > (-32767 / 2)) {

			// less than halfway to maximum 
			// only adjust lefthand servo
			
			af_st.servos[SRV_RIGHTWING] = mapRange(-32767, 0, SRV_L_MIN, SRV_L_MAX, js_st.axis[ROLL]);

		} else {
			
			af_st.servos[SRV_RIGHTWING] = mapRange(-32767, 0, SRV_L_MIN, SRV_L_MAX, js_st.axis[ROLL]);
			af_st.servos[SRV_LEFTWING] = mapRange(-32767, 0, SRV_R_MAX / 2, SRV_R_MIN / 2, js_st.axis[ROLL]);
		}

	}

	// handle PITCH + YAW

	int yaw_left = mapRange(-32767, 32767, 180, 0, js_st.axis[YAW]);
	int yaw_right = mapRange(-32767, 32767, 0, 180, js_st.axis[YAW]);
	int pitch_map = mapRange(-32767, 32767, 180, 0, js_st.axis[PITCH]);
	int combined_left = yaw_left + pitch_map;
	int combined_right = yaw_right + pitch_map;

	if(combined_left > 180) {
		
		combined_left = 180;
	
	}
	if(combined_right > 180) {

		combined_right = 180;

	}

	af_st.servos[SRV_L_ELEVRON] = combined_left;
	af_st.servos[SRV_R_ELEVRON] = combined_right;

	int throttle_esc = mapRange(-32767, 32767, 0, 255, js_st.axis[THROTTLE]);
	af_st.servos[SRV_ESC_LEFT] = throttle_esc;
	af_st.servos[SRV_ESC_RIGHT] = throttle_esc;

	af_st.servos[SRV_CAM_PAN] = mapRange(-32767, 32767, 0, 180, js_st.axis[CAM_PAN]);
	af_st.servos[SRV_CAM_TILT] = mapRange(-32767, 32767, 0, 180, js_st.axis[CAM_TILT]);

	for(x = 0 ; x < BUTTON_COUNT ; x++) {

		af_st.buttons[x] = js_st.button[x];

	}

}

void sendCtrlUpdate (int signum) {

	int x;
	char *xbee_msg;
	int msg_wrote = 0;
	char pins[3];

	pins[0] = 0x00;
	pins[1] = 0x00;
	pins[2] = 0x00;

	xbee_msg = calloc(MSG_SIZE, sizeof(char));

	xbee_msg = strcpy(xbee_msg, CMD_PREFIX);
	for(x = 0 ; x < SERVO_COUNT ; x++) {
		
		xbee_msg = strcat(xbee_msg, af_st.servos[x]);

	}

	for(x = 0 ; x < 3 ; x++) {

		pins[x] = pins[x] | (af_st.buttons[(x * 4)] & 2) << 6;
		pins[x] = pins[x] | (af_st.buttons[(x * 4) + 1] & 2) << 4;
		pins[x] = pins[x] | (af_st.buttons[(x * 4) + 2] & 2) << 2;
		pins[x] = pins[x] | (af_st.buttons[(x * 4) + 3]);

	}

	xbee_msg = strcat(xbee_msg, pins[0]);
	xbee_msg = strcat(xbee_msg, pins[1]);
	xbee_msg = strcat(xbee_msg, pins[2]);
	
	msg_wrote = write(xbee_port, xbee_msg, MSG_SIZE);
	if(msg_wrote != cmd_size) {
				
		printf("error writing to XBee, wrote: %d, cmd_size: %d\n", msg_wrote, cmd_size);

	}

}
