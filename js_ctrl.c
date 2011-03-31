/* js_ctrl.c - Paul Mandal (paul.mandal@gmail.com)
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
- Relay PPZ -> UAV
- Relay UAV -> PPZ
- Force feedback on RSSI weak/loss

Adruino:

- Handle digital pins/buttons
- Handle 3-way switch (or servo it?)
- Relay PPZ<->GCS

*/

/* Includes */

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
#include <termios.h>
#include <time.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "axbtnmap.h"

#include <signal.h>

/* Definitions */

#define VERSION_MAJOR 2       // Version information, Major #
#define VERSION_MINOR 0       // Minor #
#define VERSION_MOD   0       // Mod #

#define MSG_SIZE_CTRL 14      // Length of control update messages
#define MSG_SIZE_SYNC 14      // Length of sync messages
#define MSG_SIZE_PPZ  64      // Length of message from PPZ
#define MSG_SIZE_CFG  64      // Length of configuration message
#define MSG_BEGIN     0xFF    // Begin of control message indicator byte
#define MSG_TYPE_CTRL 0x01    // Control update message type indicator
#define MSG_TYPE_CFG  0x02    // Configuration update
#define MSG_TYPE_PPZ  0x03    // Message from PPZ
#define MSG_TYPE_SYNC 0xFE    // Sync message type indicator

#define NAME_LENGTH 128       // Length of Joystick name
#define PPM_INTERVAL 20000    // Interval for state send message
#define JS_DISCARD_UNDER 5000 // Discard joystick inputs under this level (joystick is very sensitive)
#define CAM_PAN 4             // Camera Pan axis #
#define CAM_TILT 5            // Camera Tilt axis #
#define ROLL 0                // Roll axis #
#define PITCH 1               // Pitch axis #
#define YAW 3                 // Yaw axis #
#define THROTTLE 2            // Throttle axis #

#define SERVO_COUNT 8         // Total servos on airframe
#define BUTTON_COUNT 12       // Buttons on joystick

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

#define PPZ_MSG_SIZE_CTRL 1024

#define SRV_ESC_LEFT 0
#define SRV_ESC_RIGHT 1
#define SRV_LEFTWING 2
#define SRV_RIGHTWING 3
#define SRV_L_ELEVRON 4
#define SRV_R_ELEVRON 5
#define SRV_CAM_PAN 6
#define SRV_CAM_TILT 7

#define CONFIG_FILE "js_ctrl.rc"  // Config file name
#define CONFIG_FILE_MIN_COUNT 4   // # of variables stored in config file 

/* Structures */

struct jsState {  // Store the axis and button states globally accessible
	int *axis;
	char *button;	
};

struct afState { // Store the translated (servo + buttons) states, globally accessible

	unsigned int servos[8];
	unsigned int buttons[12];

};

/* Let's do sum prototypes! */

// let's do sum prototypes!

int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh);
void sendCtrlUpdate (int signum);
void readJoystick(int jsPort);
int openPort(char *portName, char *use);
int openJoystick(char *portName);
int doHandshake(int xbeePort);
int readConfig(char **xbeePortFile, char **ppzPortFile, char **joystickPortFile, char **joystickEventFile);
void setupTimer();
void translateJStoAF();
void printState();
void initAirframe();
void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize);
char *fgetsNoNewline(char *s, int n, FILE *stream);

/* Global configuration info */

int toggleButtons[12] = {  // Set which buttons are toggleable
1, 0, 0, 0, 
0, 0, 0, 0, 
0, 0, 0, 0
};

int buttonStateCount[12] = {  // Set which buttons are multiple-state
0, 0, 0, 0, 
0, 0, 0, 0, 
0, 0, 0, 0
};

unsigned long dpadPressTime[4] = {  // Store last DPad button press time
0,  // DPad right press time
0,  // DPad left press time
0,  // DPad up press time
0   // DPad down press time
};

/* Global state storage variables */

// global variables to store states

struct afState airframeState;  // Current airframe state
struct jsState joystickState;        // Current joystick state
unsigned char axes = 2;       // # of joystick axes
unsigned char buttons = 2;    // # of joystick buttons
int oldThrottle = 0;         // Current/old throttle setting
int ppzPort;                 // PPZ port FD
int xbeePort;                // XBee port FD

/* Main function */

int main (int argc, char **argv)
{
	int jsPort;
	char xbeeBuffer[256];
	char ppzBuffer[256];
	char *joystickEventFile = NULL;
	char *joystickPortFile = NULL;
	char *xbeePortFile = NULL;
	char *ppzPortFile = NULL;

	printf("Starting js_crl version %d.%d.%d...\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD);  // Print version information

	srand(time(NULL));
	
	if(readConfig(&xbeePortFile, &ppzPortFile, &joystickPortFile, &joystickEventFile) < 0) { // Read our config into our config vars

		perror("js_ctrl"); // Error reading config file
		return 1;

	}

	initAirframe();  // Init airframe state

	if((xbeePort = openPort(xbeePortFile, "XBee")) < 0) { // open the XBee port

		return 1;
	}

	/*if((ppzPort = openPort(ppzPortFile, "PPZ")) < 0) { // open the PPZ port
		return 1;
	}*/

	if((jsPort = openJoystick(joystickPortFile)) < 0) { // open the Joystick
		return 1;
	}

	if(doHandshake(xbeePort) < 0) { // do handshaking w/ Arduino
		return 1;
	}

	setupTimer(); 	// Set up timer (every 20ms)

	printf("Ready to read JS & relay for PPZ...\n");

	joystickState.axis = calloc(axes, sizeof(int));        // Allocate memory for joystick axes
	joystickState.button = calloc(buttons, sizeof(char));  // Allocate memory for joystick buttons

	while (1) {

		readJoystick(jsPort);  // Check joystick for updates

		translateJStoAF();	// update Airframe model

		//printState(); 	// print JS & AF state

		while(read(xbeePort, &xbeeBuffer, 256) > 0) {  // check XBee port

			printf("XBee: %s\n", xbeeBuffer);

		}

		/*while(read(ppzPort, &ppzBuffer, 256) > 0) {  // check PPZ port

			
				printf("PPZ:  %s\n", ppzBuffer);

		}*/

		usleep(1);
	}

	return 0;

}

/* Function definitions */

/* initAirframe() - Initalise airframe state */

void initAirframe() {

	int i;

	for(i = 0 ; i < 8 ; i++) {

		airframeState.servos[i] = 0;  // Set all servo pos to 0

	}

}

/* map() - Map a number in inRangeLow->inRangeHigh range into outRangeLow->outRangeHigh */

int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh)
{
	return outRangeLow + (value-inRangeLow)*(outRangeHigh-outRangeLow)/(inRangeHigh-inRangeLow);
}

/* openPort(portName, use) - Open a UART portName for usage use */

int openPort(char *portName, char *use) {

	int fd;
	printf("Opening serial port %s for %s..\n", portName, use);

	if ((fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {  // Try to open the port
		perror("js_ctrl");  // If there's an error, blow up!
		return -1;
	}

	struct termios options;  // The port opened, set it up the port

	tcgetattr(fd, &options);              // Get current settings
	cfsetispeed(&options, B38400);        // Set input speed to 38400
	cfsetospeed(&options, B38400);        // Set output speed to 38400
	options.c_cflag |= (CLOCAL | CREAD);  // Set sum flags (CLOCAL & CREAD)
	tcsetattr(fd, TCSANOW, &options);     // Set options

	return fd; // Return the file descriptor for our port

}

/* openJoystick() - Open the joystick port portName */

int openJoystick(char *portName) {

	uint16_t btnmap[BTNMAP_SIZE];
	uint8_t axmap[AXMAP_SIZE];
	int btnmapok = 1;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";
	int fd, i;

	printf("Opening joystick %s..\n", portName);
	if ((fd = open(portName, O_RDONLY | O_NONBLOCK)) < 0) {  // Open joystick port in non-blocking mode
		perror("js_ctrl");  // Error opening port
		return -1;
	}

	ioctl(fd, JSIOCGVERSION, &version);        // Get the joystick driver version
	ioctl(fd, JSIOCGAXES, &axes);              // Get the axes count
	ioctl(fd, JSIOCGBUTTONS, &buttons);        // Get the button count
	ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);  // Get the joystick name

	getaxmap(fd, axmap);   // Get the axis map
	getbtnmap(fd, btnmap); // Get the button map

	printf("Driver version is %d.%d.%d.\n",
		version >> 16, (version >> 8) & 0xff, version & 0xff); // Display driver version to user

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
		printf("Joystick (%s) initialised with %d axes and %d buttons.\n", name, axes, buttons);  // Button map is OK, print joystick info
	}

	return fd;  // Return joystick file descriptor

}

/* doHandshake() - handshake with receiver */

int doHandshake(int xbeePort) {

	unsigned char handshakeMsg[MSG_SIZE_SYNC];
	unsigned int checksum;
	unsigned char handshakeAck[MSG_SIZE_SYNC];
	unsigned char testChar;
	unsigned char msgType;
	int i, handshook = 0;

	handshakeMsg[0] = MSG_BEGIN;
	handshakeMsg[1] = MSG_TYPE_SYNC;
	for(i = 2 ; i < (MSG_SIZE_SYNC - 1) ; i++) {

		handshakeMsg[i] = rand() % 255;  // Build the handshake msg, it is MSG_BEGIN + MSG_TYPE_SYNC + random characters + checksum
		
	}

	checksum = 0x00;
	for(i = 0 ; i < (MSG_SIZE_SYNC - 1) ; i++) {

		checksum = checksum ^ (unsigned int)handshakeMsg[i];  // Build the checksum

	}

	handshakeMsg[MSG_SIZE_SYNC - 1] = (unsigned char)checksum & 0xFF;  // Store the checksum

	printf("Handshaking..");

	while(!handshook) {

		//printf(".");
		fflush(stdout);
		writePortMsg(xbeePort, "XBee", handshakeMsg, MSG_SIZE_SYNC);  // Write the handshake to the XBee port
		usleep(20000);                                           // Give 20ms to respond
		
		i = 0;
		while(read(xbeePort, &testChar, 1) == 1) {

			if(testChar == MSG_BEGIN) { // This is a begin message, let's check the type
	
				handshakeAck[i] = testChar; // Store the begin msg byte
				i++;
		
				while(read(xbeePort, &msgType, 1) != 1) {} // Try to read until we get something (DEBUG: may want to add a timer to break this
				if(msgType == MSG_TYPE_SYNC) {  // This is a sync msg

					handshakeAck[i] = msgType;
					i++;
					for(i = 2 ; i < MSG_SIZE_SYNC ; i++) {

						read(xbeePort, &testChar, 1);
						handshakeAck[i] = testChar;  // Read the rest of the sync msg into our buffer

					}

					checksum = 0x00;
					for(i = 0 ; i < MSG_SIZE_SYNC ; i++) {

						checksum = checksum ^ (unsigned int)handshakeAck[i];  // Verify the checksum
	
					}
					if(checksum == 0x00) {

						handshook = 1;  // Sync ack was valid

					}
					

				}
			}

		}

		while(read(xbeePort, &handshakeAck, 1) > 0) {  // Discard XBee port buffer since it might have junk in it
		}

	}

	printf("got ACK, handshake complete!\n");

	return 1;	

}

// setupTimer() - set up pulse timer
void setupTimer() {

	struct sigaction sa;
	struct itimerval timer;
	
	printf("Setting up timer..\n");
	memset (&sa, 0, sizeof (sa));              // Make signal object
	sa.sa_handler = &sendCtrlUpdate;           // Set signal function handler in 'sa'
	sigaction (SIGALRM, &sa, NULL);            // Set SIGALRM signal handler

	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = PPM_INTERVAL;     // Set timer interval to 20000usec (20ms)
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = PPM_INTERVAL;  // Set timer reset to 20000usec (20ms)

	printf("Starting pulse timer..\n");
	setitimer (ITIMER_REAL, &timer, NULL);     // Start the timer

}

/* readJoystick(jsPort) - Read joystick state from jsPort, update joystickState */

void readJoystick(int jsPort) {

	struct js_event js;
	int jsValue;

	while(read(jsPort, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:

				if(toggleButtons[js.number] == 1) {  // Check if the button is a toggle or not

					if(js.value == 1) { // If it is a toggle and this event is a button press, toggle the button

						joystickState.button[js.number] = !joystickState.button[js.number];

					}
				
				} else if(buttonStateCount[js.number] > 0) {  // Check if the button has multiple states

					if(js.value == 1) {  // If it is a multi-state and this event is a button press, cycle through states

						if(joystickState.button[js.number] < buttonStateCount[js.number]) {

							joystickState.button[js.number] = joystickState.button[js.number] + 1;	

						} else {

							joystickState.button[js.number] = 0;  // Max state hit, zero out state

						}

					}

				} else {

					joystickState.button[js.number] = js.value;  // Normal button, switch to value from event

				}


				break;
			case JS_EVENT_AXIS:

				if(abs(js.value) > JS_DISCARD_UNDER) {  // If the value is greater than our discard value set it
			
					jsValue = js.value;
					
				} else {

					jsValue = 0;  // Disregard values less than JS_DISCARD_UNDER to avoid excessively sensitive sticks

				}

				if(js.number == THROTTLE) { // Handle throttle axis

					jsValue = jsValue * -1;  // Invert the throttle axis

					if(jsValue > 0) {  // Check if the joystick is in the "up" or "down" section of the axis

						if(jsValue > oldThrottle) {

							joystickState.axis[js.number] = jsValue;  // Joystick is in the up section and has passed previous max throttles
							oldThrottle = jsValue;

						}

					} else {

						jsValue = jsValue + 32767;   // Since we inverted the axis, add the MAX_VAL to this
						if(jsValue < oldThrottle) {  // Joystick is in the down section and has passed the previous min throttle

							joystickState.axis[js.number] = jsValue;
							oldThrottle = jsValue;

						}

					}
					
				} else {

					joystickState.axis[js.number] = jsValue;  // Regular axis, just store the current value

				}
				break;

			}

		}

}

/* translateJStoAF() - Translate current joystick settings to airframe settings (e.g. axis -> servos) */

void translateJStoAF() {

	int x;
	if(joystickState.axis[ROLL] > 0) {

		if(joystickState.axis[ROLL] < (32767 / 2)) {

			// less than halfway to maximum 
			// only adjust lefthand servo
			
			airframeState.servos[SRV_LEFTWING] = map(joystickState.axis[ROLL], 0, 32767, SRV_L_MIN, SRV_L_MAX);

		} else {
			
			airframeState.servos[SRV_LEFTWING] = map(joystickState.axis[ROLL], 0, 32767, SRV_L_MIN, SRV_L_MAX);
			airframeState.servos[SRV_RIGHTWING] = map(joystickState.axis[ROLL], 0, 32767, SRV_R_MAX / 2, SRV_R_MIN / 2);

		}

	} else {

		if(joystickState.axis[ROLL] > (-32767 / 2)) {

			// less than halfway to maximum 
			// only adjust lefthand servo
			
			airframeState.servos[SRV_RIGHTWING] = map(joystickState.axis[ROLL], -32767, 0, SRV_L_MIN, SRV_L_MAX);

		} else {
			
			airframeState.servos[SRV_RIGHTWING] = map(joystickState.axis[ROLL], -32767, 0, SRV_L_MIN, SRV_L_MAX);
			airframeState.servos[SRV_LEFTWING] = map(joystickState.axis[ROLL], -32767, 0, SRV_R_MAX / 2, SRV_R_MIN / 2);
		}

	}

	// handle PITCH + YAW

	int yawLeft = map(joystickState.axis[YAW], -32767, 32767, 180, 0);
	int yawRight = map(joystickState.axis[YAW], -32767, 32767, 0, 180);
	int pitchMap = map(joystickState.axis[PITCH], -32767, 32767, 180, 0);
	int combinedLeft = yawLeft + pitchMap;
	int combinedRight = yawRight + pitchMap;

	if(combinedLeft > 180) {
		
		combinedLeft = 180;
	
	}
	if(combinedRight > 180) {

		combinedRight = 180;

	}

	airframeState.servos[SRV_L_ELEVRON] = combinedLeft;
	airframeState.servos[SRV_R_ELEVRON] = combinedRight;

	int throttle_esc = map(joystickState.axis[THROTTLE], 0, 32767, 0, 254);
	airframeState.servos[SRV_ESC_LEFT] = throttle_esc;
	airframeState.servos[SRV_ESC_RIGHT] = throttle_esc;

	airframeState.servos[SRV_CAM_PAN] = map(joystickState.axis[CAM_PAN], -32767, 32767, 0, 180);
	airframeState.servos[SRV_CAM_TILT] = map(joystickState.axis[CAM_TILT], -32767, 32767, 0, 180);

	for(x = 0 ; x < BUTTON_COUNT ; x++) {

		airframeState.buttons[x] = joystickState.button[x];

	}

}

/* sendCtrlUpdate() - Send latest control state to XBee port */

void sendCtrlUpdate (int signum) {

	int x;
	unsigned char xbeeMsg[MSG_SIZE_CTRL];
	unsigned int checksum;
	
	xbeeMsg[0] = MSG_BEGIN;      // First character of xbeeMsg is MSG_BEGIN
	xbeeMsg[1] = MSG_TYPE_CTRL;  // Message type = control
	for(x = 0 ; x < SERVO_COUNT ; x++) {

		xbeeMsg[x + 2] = (unsigned char)airframeState.servos[x];  // Next 8 bytes are servo states

	}

	for(x = 0 ; x < 3 ; x++) {  // Next 3 bytes are 12 buttons, 2 bits per button

		xbeeMsg[x + 10] = (airframeState.buttons[(x * 4)] & 3) << 6;                       // Mask away anything but the last 2 bits and then bitshift to the left
		xbeeMsg[x + 10] = xbeeMsg[x + 10] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
		xbeeMsg[x + 10] = xbeeMsg[x + 10] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
		xbeeMsg[x + 10] = xbeeMsg[x + 10] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

	}

	checksum = 0x00;
	for(x = 0 ; x < (MSG_SIZE_CTRL - 1); x++) {

		checksum = checksum ^ (unsigned int)xbeeMsg[x]; // Generate checksum

	}
	xbeeMsg[MSG_SIZE_CTRL - 1] = (unsigned char)checksum & 0xFF; // Last byte is checksum

	writePortMsg(xbeePort, "XBee", xbeeMsg, MSG_SIZE_CTRL);
}

/* writePortMsg(outputPort, portName, message, messageSize) - Write message to outputPort, deliver error if message fails to write */

void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize) {

	int msgWrote = 0;
	msgWrote = write(outputPort, message, messageSize);  // write() and store written byte count in msgWrote
	if(msgWrote != messageSize) { // If written byte count is not expected value
				
		printf("error writing to %s, wrote: %d/%d bytes.\n", portName, msgWrote, messageSize);  // Output error and info on what happened

	}


}

/* printState() - Debug function, prints the current joystick and airframe states */

void printState() {

	int i;
	printf("\r");

	if (axes) {
		printf("Axes: ");
		for (i = 0; i < axes; i++)
			printf("%2d:%6d ", i, joystickState.axis[i]);  // Print all joystick axes
	}

	printf("Servos: ");
	for(i = 0 ; i < SERVO_COUNT ; i++) {

		printf("%2d:%03d ", i, airframeState.servos[i]);  // Print all airframe servos

	}

	printf("Pins: ");
	for(i = 0 ; i < BUTTON_COUNT ; i++) {

		printf("%2d:%d ", i, airframeState.buttons[i]);  // Print all button states

	}

	fflush(stdout);

}

/* readConfig() - Read values from configuration file */

int readConfig(char **xbeePortFile, char **ppzPortFile, char **joystickPortFile, char **joystickEventFile) {

	FILE *fp;
	int readCount = 0, lineBuffer = 1024;
	char line[lineBuffer];
	if ((fp = fopen(CONFIG_FILE, "r")) == NULL) {  // Open the config file read-only
		
		return -1;  // Return -1 if error opening

	} else {

		while (fgetsNoNewline(line, lineBuffer, fp) != NULL) {  // Read the entire file checking it line by line

			if(strcmp(line, "[XBee Port File]") == 0) {  // Line matches a config variable header, read the next line (value) and store it
		
				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {
					
					*xbeePortFile = calloc(strlen(line) + 1, sizeof(char));  // Allocate memory for our variable
					strcpy(*xbeePortFile, line);                             // Copy value into our var
					readCount++;                                            // Increment our value count

				}

			} else if(strcmp(line, "[PPZ Port File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					*ppzPortFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(*ppzPortFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Port File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					*joystickPortFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(*joystickPortFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Event File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					*joystickEventFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(*joystickEventFile, line);
					readCount++;

				}

			}

		}

	}

	fclose(fp);

	printf("\nUsing config from %s:\n", CONFIG_FILE); // Print config values
	printf("       XBee Port: %s\n", *xbeePortFile);
	printf("        PPZ Port: %s\n", *ppzPortFile);
	printf("   Joystick Port: %s\n", *joystickPortFile);
	printf("  Joystick Event: %s\n\n", *joystickEventFile);

	if(readCount >= CONFIG_FILE_MIN_COUNT) {

		return 1;  // Read enough config variables, success

	} else {

		return -1;  // Didn't read enough of config values

	}	

}

/* fgetsNoNewline() - Wrapper for fgets() that returns a string without the newline */

char *fgetsNoNewline(char *s, int n, FILE *stream) {

	char *newLine;
	if(fgets(s, n, stream) != NULL) {

		if((newLine = strrchr(s, '\n')) != NULL) {

			*newLine = '\0';

		}
		return s;

	} else {

		return NULL;

	}

}
