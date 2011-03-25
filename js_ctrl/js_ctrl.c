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

- DPad -> servo control
- Throttle control -> servo
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

int updated = 0;

#define NAME_LENGTH 128
#define PPM_INTERVAL 100000
#define JS_DISCARD_UNDER 5000
#define CMD_PREFIX "Z"
#define CMD_SUFFIX "X"
#define CAM_PAN 4
#define CAM_TILT 5
#define ROLL 0
#define PITCH 1
#define YAW 3
#define THROTTLE 2

#define SERVO_COUNT 6
#define ESC_COUNT 2
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

struct js_state {
		int *axis;
		char *button;	
};

struct airframe_state {

	int servos[8];

};

// global vars to keep state

struct js_state js_st;
struct airframe_state af_st;
unsigned char axes = 2;
unsigned char buttons = 2;
int test_mode = 0;
int ppz_port;
int xbee_port;

// let's do sum prototypes!

int mapRange(int a1,int a2,int b1,int b2,int s);
void send_ctrl_update (int signum);


/* Timer stuff */

void send_ctrl_update (int signum)
{

	// generate and send output string

	// possible:

	// pulse but no change: PREFIX-X-SUFFIX
	// changes: S[##][DEG] for each servo
	// P[##][0/1] for each digital pin

	// translate js_state -> airframe_state

	int i;
	char *xbee_msg;
	int msg_size = 0;
	int cmd_style = 1;
	int msg_wrote = 0;

	if(updated == 1) {

		// SERVOS: 6x
		// 1x per wing
		// 1x per elevron
		// 2x on cam
		// ESC: 2x
		// 1x per engine (1x per wing)
		// PINS: (buttons) many
		// yep
		
		// translate ROLL into servo values
		
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

		char buffer[4];
	
		int throttle_esc = mapRange(-32767, 32767, 0, 255, js_st.axis[THROTTLE]);
		af_st.servos[SRV_ESC_LEFT] = throttle_esc;
		af_st.servos[SRV_ESC_RIGHT] = throttle_esc;

		af_st.servos[SRV_CAM_PAN] = mapRange(-32767, 32767, 0, 180, js_st.axis[CAM_PAN]);
		af_st.servos[SRV_CAM_TILT] = mapRange(-32767, 32767, 0, 180, js_st.axis[CAM_TILT]);

		// build XBee msg

		msg_size = strlen(CMD_PREFIX) + (SERVO_COUNT + ESC_COUNT * 6) + (buttons * 4) + strlen(CMD_SUFFIX) + 1;
		// DEBUG: buttons only
		//msg_size = strlen(CMD_PREFIX) + (buttons * 4) + strlen(CMD_SUFFIX) + 1;

		int cmd_size = strlen(CMD_PREFIX) + strlen(CMD_SUFFIX) + 7;

		char *xbee_cmd;

		if(cmd_style == 1) {

			xbee_cmd = calloc(cmd_size, sizeof(char));
			for(i = 0 ; i < 1 ; i++) {

				xbee_cmd = strcpy(xbee_cmd, CMD_PREFIX);
				sprintf(xbee_cmd, "%sS%02d%03d%s", CMD_PREFIX, i, af_st.servos[i], CMD_SUFFIX);
				msg_wrote = write(xbee_port, xbee_cmd, cmd_size);
				if(msg_wrote != cmd_size) {
				
					printf("error writing to XBee, wrote: %d, cmd_size: %d\n", msg_wrote, cmd_size);

				}

			}

			for(i = 0 ; i < 1 ; i++) {

				xbee_cmd = strcpy(xbee_cmd, CMD_PREFIX);
				sprintf(xbee_cmd, "%sP%02d%03d%s", CMD_PREFIX, i, js_st.button[i], CMD_SUFFIX);
				msg_wrote = write(xbee_port, xbee_cmd, cmd_size);
				if(msg_wrote != cmd_size) {
				
					printf("error writing to XBee, wrote: %d, cmd_size: %d\n", msg_wrote, cmd_size);

				}

			}
				


 		} else {

			xbee_msg = calloc(msg_size, sizeof(char));

			xbee_msg = strcpy(xbee_msg, CMD_PREFIX);

			// build all servos
	
			// S01 - left wing
		
			sprintf(buffer, "%03d", af_st.servos[SRV_LEFTWING]);
			//itoa(af_st.servos[SRV_LEFTWING], buffer, 10);

			xbee_msg = strcat(xbee_msg, "S01");
			xbee_msg = strcat(xbee_msg, buffer);

			// S02 - right wing

			sprintf(buffer, "%03d", af_st.servos[SRV_RIGHTWING]);

			xbee_msg = strcat(xbee_msg, "S02");
			xbee_msg = strcat(xbee_msg, buffer);

			// S03 - left elevron

			sprintf(buffer, "%03d", af_st.servos[SRV_L_ELEVRON]);

			xbee_msg = strcat(xbee_msg, "S03");
			xbee_msg = strcat(xbee_msg, buffer);

			// S04 - right elevron

			sprintf(buffer, "%03d", af_st.servos[SRV_R_ELEVRON]);

			xbee_msg = strcat(xbee_msg, "S04");
			xbee_msg = strcat(xbee_msg, buffer);

			// S05 - cam pan

			sprintf(buffer, "%03d", af_st.servos[SRV_CAM_PAN]);

			xbee_msg = strcat(xbee_msg, "S05");
			xbee_msg = strcat(xbee_msg, buffer);

			// S06 - cam tilt

			sprintf(buffer, "%03d", af_st.servos[SRV_CAM_TILT]);

			xbee_msg = strcat(xbee_msg, "S06");
			xbee_msg = strcat(xbee_msg, buffer);

			// build all ESCs

			// E01 - left ESC
			sprintf(buffer, "%03d", af_st.servos[SRV_ESC_LEFT]);
	
			xbee_msg = strcat(xbee_msg, "E01");
			xbee_msg = strcat(xbee_msg, buffer);
	
			// E02 - right ESC
			sprintf(buffer, "%03d", af_st.servos[SRV_ESC_RIGHT]);

			xbee_msg = strcat(xbee_msg, "E02");
			xbee_msg = strcat(xbee_msg, buffer);

			// build all pins / buttons
	
			// DEBUG: buttons only
			//strcpy(xbee_msg, CMD_PREFIX);

			for(i = 0 ; i < buttons ; i++) {
	
				xbee_msg = strcat(xbee_msg, "P");
				sprintf(buffer, "%02d", i);
				xbee_msg = strcat(xbee_msg, buffer);
	
				sprintf(buffer, "%d", js_st.button[i]);
				xbee_msg = strcat(xbee_msg, buffer);
	
			}

			xbee_msg = strcat(xbee_msg, CMD_SUFFIX);
			updated = 0;

		}

	}  else {

		msg_size = strlen(CMD_PREFIX) + 4 + strlen(CMD_SUFFIX);
		xbee_msg = calloc(msg_size, sizeof(char));
		xbee_msg = strcpy(xbee_msg, CMD_PREFIX);
		xbee_msg = strcat(xbee_msg, "_X_");
		xbee_msg = strcat(xbee_msg, CMD_SUFFIX);

	}

	// write XBee msg
	if(test_mode == 1) {
		
		printf("%s\n", xbee_msg);
	
	}

	if(cmd_style != 1) {

		msg_wrote = write(xbee_port, xbee_msg, msg_size);
		if(msg_wrote != msg_size) {

			printf("error writing to XBee, wrote: %d, msg_size: %d\n", msg_wrote, msg_size);

		}

	}

	// update servos controlled by dpad

	/*if(dpad_state[0] > 0) {

		if(js_st.axis[DPAD_X] < 180) {
		}

	}*/

	//free(xbee_msg);

	if(test_mode == 2) {

		printf("\r");

		printf("Roll: %6d Pitch: %6d Yaw: %6d Throttle: %6d Pan: %6d Tilt: %6d -- ", js_st.axis[ROLL], js_st.axis[PITCH], js_st.axis[YAW], js_st.axis[THROTTLE], js_st.axis[CAM_PAN], js_st.axis[CAM_TILT]);	

		printf("AP: [%d] ", js_st.button[0]);

		if (buttons) {
			for (i = 1; i < buttons; i++)
				printf("%2d:[%d] ", i, js_st.button[i]);
		}

		fflush(stdout);
		
	}

}

int mapRange(int a1,int a2,int b1,int b2,int s)
{
	return b1 + (s-a1)*(b2-b1)/(a2-a1);
}

int main (int argc, char **argv)
{
	int fd, i;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";
	uint16_t btnmap[BTNMAP_SIZE];
	uint8_t axmap[AXMAP_SIZE];
	int btnmapok = 1;
	struct js_event js;
	char xbee_buffer[PPZ_MSG_SIZE];
	char ppz_buffer[PPZ_MSG_SIZE];
	int js_value;
	int old_throttle = 0;
	int first = 1;

	// init airframe

	for(i = 0 ; i < 8 ; i++) {

		af_st.servos[i] = 0;

	}

	// timer stuff

	struct sigaction sa;
	struct itimerval timer;

	if (argc < 4 || !strcmp("--help", argv[1])) {
		puts("");
		puts("Usage: js_ctrl <XBee UART> <PPZ UART> <joystick device>");
		puts("");
		return 1;
	}

	if(argc > 4) {

		test_mode = 1;

	}

	printf("Test mode: %d\n", test_mode);
	printf("Opening XBee serial port %s..\n", argv[argc - 3]);

	if ((xbee_port = open(argv[argc - 3], O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("js_ctrl");
		return 1;
	}

	struct termios options;

	tcgetattr(xbee_port, &options);
	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);
	options.c_cflag |= (CLOCAL | CREAD);
	tcsetattr(xbee_port, TCSANOW, &options);

	/*printf("Opening PPZ serial port %s..\n", argv[argc - 2]);

	if ((ppz_port = open(argv[argc - 2], O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
		perror("js_ctrl");
		return 1;
	}

	tcgetattr(ppz_port, &options);
	cfsetispeed(&options, B38400);
	cfsetospeed(&options, B38400);
	options.c_cflag |= (CLOCAL | CREAD);
	tcsetattr(ppz_port, TCSANOW, &options);*/

	printf("Opening joystick %s..\n", argv[argc - 1]);
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

	printf("Setting up timer..\n");
	memset (&sa, 0, sizeof (sa));
	sa.sa_handler = &send_ctrl_update;
	sigaction (SIGALRM, &sa, NULL);

	timer.it_value.tv_sec = 1;
	timer.it_value.tv_usec = 0;
	timer.it_interval.tv_sec = 1;
	timer.it_interval.tv_usec = 0;
	setitimer (ITIMER_REAL, &timer, NULL);

	printf("Ready to read JS & relay for PPZ...\n");

	js_st.axis = calloc(axes, sizeof(int));
	js_st.button = calloc(buttons, sizeof(char));

	while (1) {

		// check joystick
		if (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:

				// check if the button is a toggle or not
				if(toggle_buttons[js.number] == 1) {

					if(js.value == 1) {

						updated = 1;
						js_st.button[js.number] = !js_st.button[js.number];

					}

				// check if the button has multiple states, if so, cycle through them				

				} else if(button_state_count[js.number] > 0) {

					if(js.value == 1) {

						updated = 1;

						if(js_st.button[js.number] < button_state_count[js.number]) {

							js_st.button[js.number] = js_st.button[js.number] + 1;	

						} else {

							js_st.button[js.number] = 0;

						}

					}

				// normal button, switch to value received by event

				} else {
		
					updated = 1;
					js_st.button[js.number] = js.value;

				}


				break;
			case JS_EVENT_AXIS:
				
				// disregard small values (probably stuck sticks)

				if(abs(js.value) > JS_DISCARD_UNDER) {
			
					updated = 1;
					js_value = js.value;
					
				} else {

					if(js_st.axis[js.number] != 0) {

						updated = 1;

					}
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

		// check XBee port

		while(read(xbee_port, &xbee_buffer, 1) > 0) {

			if(first == 1) {

				printf("Read from XBee: %s", xbee_buffer);
				first = 0;

			} else {

				printf("%s", xbee_buffer);
	
			}

		}

		// check PPZ port

		/*while(read(ppz_port, &ppz_buffer, 1) > 0) {

			if(first == 1) {

				printf("Read from PPZ: %s", ppz_buffer);
				first = 0;

			} else {

				printf("%s", ppz_buffer);
	
			}

		}*/

		first = 0;

		usleep(1);
	}

	return 0;

}
