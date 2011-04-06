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

- Move all values to config file
- Write config msg updater
- DPad -> servo control
- Relay PPZ -> UAV
- Relay UAV -> PPZ
- Force feedback on RSSI weak/loss
- Better/propermessage handling, PPZ and Arduino

Adruino:

- Move all config values to EEPROM
- Handle config updates via msg
- Improve message handling, PPZ and JS
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

#define DEBUG_LEVEL 0	      // Debug level - tells compiler to include or exclude debug message code
			      // Debug level - 1 - Debug messaging/handshaking
			      // Debug level - 2 - Debug joystick position info

#define VERSION_MAJOR 2       // Version information, Major #
#define VERSION_MINOR 9       // Minor #
#define VERSION_MOD   1       // Mod #

#define MSG_BEGIN     0xFF    // Begin of control message indicator byte
#define MSG_TYPE_CTRL 0x01    // Control update message type indicator
#define MSG_TYPE_CFG  0x02    // Configuration update
#define MSG_TYPE_PPZ  0x03    // Message from PPZ
#define MSG_TYPE_SYNC 0xFE    // Sync message type indicator
#define MSG_BUFFER_SIZE 256   // Message buffer size in bytes
#define MSG_HEADER_SIZE 4     // Message header size in bytes
#define CMDS_PER_ACK  60      // Assume lost signal if we send this many commands without an ack

#define NAME_LENGTH 128       // Length of Joystick name
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
#define CONFIG_FILE_MIN_COUNT 8   // # of variables stored in config file 

/* Structures */

typedef struct _jsState {  // Store the axis and button states globally accessible
	int *axis;
	char *button;
	int prevLeftThrottle;
	int prevRightThrottle;	
	unsigned char axes;
	unsigned char buttons;
} jsState;

typedef struct _afState { // Store the translated (servo + buttons) states, globally accessible

	unsigned int servos[SERVO_COUNT];
	unsigned int buttons[BUTTON_COUNT];

} afState;

typedef struct _configValues {

	char *joystickEventFile; // joystickEvent filename (for RUMBLE!)
	char *joystickPortFile;  // joystickPort filename
	char *xbeePortFile;      // xbeePort filename
	char *ppzPortFile;       // ppzPort filename
	int *buttonStateCount;   // Button state counts
	int jsDiscardUnder;	 // Joystick discard under threshold
	int ppmInterval;	 // Interval to send commands to XBee
	int commandsPerAck;      // How many commands without ACK before setting lostSignal

} configValues;

typedef struct _messageState {

	unsigned char *messageBuffer;
	int readBytes;
	int length;

} messageState;

/* Let's do sum prototypes! */

int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh);
void sendCtrlUpdate (int signum);
void readJoystick(int jsPort, jsState *joystickState, configValues configInfo);
int openPort(char *portName, char *use);
int openJoystick(char *portName, jsState *joystickState);
int readConfig(configValues *configInfo);
void initTimer();
void translateJStoAF(jsState joystickState);
void printState(jsState joystickState);
void initAirframe();
void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize);
char *fgetsNoNewline(char *s, int n, FILE *stream);
int checkMessages(int msgPort, messageState *msg);
int testChecksum(unsigned char *message, int length);
unsigned char generateChecksum(unsigned char *message, int length);
void checkSignal(int commandsPerAck);
void processMessage(unsigned char *message, int length);

/* Global configuration info */

unsigned long dpadPressTime[4] = {  // Store last DPad button press time
0,  // DPad right press time
0,  // DPad left press time
0,  // DPad up press time
0   // DPad down press time
};

/* Global state storage variables */

// global variables to store states

afState airframeState; // Current airframe state
int ppzPort;           // PPZ port FD
int xbeePort;          // XBee port FD

volatile int commandsSinceLastAck = 0;  // Commands sent since last ACK
volatile int handShook = 0;             // Handshook?

#if DEBUG_LEVEL > 0
int debugCommandsPerAck = 0;
#endif

/* Main function */

int main (int argc, char **argv)
{
	int jsPort;  // JoystickPort FD
	jsState joystickState;                // Current joystick state
	configValues configInfo;   	      // Configuration values
	char *joystickEventFile = NULL;       // joystickEvent filename (for RUMBLE!)
	char *joystickPortFile = NULL;        // joystickPort filename
	char *xbeePortFile = NULL;            // xbeePort filename
	char *ppzPortFile = NULL;             // ppzPort filename
	struct jsState joystickState;         // Current joystick state
	messageState xbeeMsg;		      // messageState for incoming XBee message

	xbeeMsg.messageBuffer = calloc(MSG_BUFFER_SIZE, sizeof(char));
	xbeeMsg.readBytes = 0;
	xbeeMsg.length = MSG_HEADER_SIZE; // Init message.length as header length size

	printf("Starting js_crl version %d.%d.%d...\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD);  // Print version information

	srand(time(NULL));  // Init random using current time
	
	if(readConfig(&configInfo) < 0) { // Read our config into our config vars

		perror("js_ctrl"); // Error reading config file
		return 1;

	}

	initAirframe();  // Init airframe state

	if((xbeePort = openPort(configInfo.xbeePortFile, "XBee")) < 0) { // open the XBee port

		return 1;
	}

	/*if((ppzPort = openPort(ppzPortFile, "PPZ")) < 0) { // open the PPZ port
		return 1;
	}*/

	if((jsPort = openJoystick(configInfo.joystickPortFile, &joystickState)) < 0) { // open the Joystick
		return 1;
	}

	initTimer(); 	// Set up timer (every 20ms)

	printf("Ready to read JS & relay for PPZ...\n");

	while (1) {

		if(!handShook) {

			printf("Handshaking..");
			#if DEBUG_LEVEL == 1	
			printf("\n");
			#endif

			while(!handShook) {  // Handshaking loop

				if(!checkMessages(xbeePort, &xbeeMsg)) { // Check for pending msg bytes

					usleep(100); // If nothing is there pause for 100usec, handshake is sent out by interrupt at 50Hz

				};

			}

			printf("got ACK, handshake complete!\n");
	
		}

		readJoystick(jsPort, &joystickState, configInfo);  // Check joystick for updates

		translateJStoAF(joystickState);	// update Airframe model

		#if DEBUG_LEVEL == 2
		printState(joystickState); 	// print JS & AF state
		#endif
		
		int x;
		for(x = 0 ; x < 10 ; x++) {  // Try to read 10 bytes per loop

			if(!checkMessages(xbeePort, &xbeeMsg)) { // Check for pending msg bytes

				usleep(100); // If there was nothing pending pause for 100usec, max pause per loop is 1,000usec = 1ms
			};  

		}

		checkSignal(configInfo.commandsPerAck);  // Check if our signal is still good
	
	}

	free(xbeeMsg.messageBuffer);
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
	cfsetispeed(&options, B115200);        // Set input speed to 115200
	cfsetospeed(&options, B115200);        // Set output speed to 115200
	options.c_cflag |= (CLOCAL | CREAD);  // Set sum flags (CLOCAL & CREAD)
	tcsetattr(fd, TCSANOW, &options);     // Set options

	return fd; // Return the file descriptor for our port

}

/* openJoystick() - Open the joystick port portName */

int openJoystick(char *portName, jsState *joystickState) {

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
	ioctl(fd, JSIOCGAXES, &joystickState->axes);              // Get the axes count
	ioctl(fd, JSIOCGBUTTONS, &joystickState->buttons);        // Get the button count
	ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);  // Get the joystick name

	joystickState->axis = calloc(joystickState->axes, sizeof(int));        // Allocate memory for joystick axes
	joystickState->button = calloc(joystickState->buttons, sizeof(char));  // Allocate memory for joystick buttons

	getaxmap(fd, axmap);   // Get the axis map
	getbtnmap(fd, btnmap); // Get the button map

	printf("Driver version is %d.%d.%d.\n",
		version >> 16, (version >> 8) & 0xff, version & 0xff); // Display driver version to user

	/* Determine whether the button map is usable. */
	for (i = 0; btnmapok && i < joystickState->buttons; i++) {
		if (btnmap[i] < BTN_MISC || btnmap[i] > KEY_MAX) {
			btnmapok = 0;
			break;
		}
	}
	if (!btnmapok) {
		/* btnmap out of range for names. Don't print any. */
		puts("js_ctrl is not fully compatible with your kernel. Unable to retrieve button map!");
		printf("Joystick (%s) has %d axes ", name, joystickState->axes);
		printf("and %d buttons.\n", joystickState->buttons);
	} else {
		printf("Joystick (%s) initialised with %d axes and %d buttons.\n", name, joystickState->axes, joystickState->buttons);  // Button map is OK, print joystick info
	}

	return fd;  // Return joystick file descriptor

}

/* initTimer() - set up pulse timer */

void initTimer(configValues configInfo) {
 
	struct sigaction sa;
	struct itimerval timer;
	
	printf("Setting up timer..\n");
	memset (&sa, 0, sizeof (sa));                       // Make signal object
	sa.sa_handler = &sendCtrlUpdate;                    // Set signal function handler in 'sa'
	sigaction (SIGALRM, &sa, NULL);                     // Set SIGALRM signal handler

	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = configInfo.ppmInterval;    // Set timer interval to 20000usec (20ms)
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = configInfo.ppmInterval; // Set timer reset to 20000usec (20ms)

	printf("Starting pulse timer..\n");
	setitimer(ITIMER_REAL, &timer, NULL);               // Start the timer

}

/* readJoystick(jsPort, joystickState) - Read joystick state from jsPort, update joystickState */

void readJoystick(int jsPort, jsState *joystickState, configValues configInfo) {

	struct js_event js;
	int jsValue;

	while(read(jsPort, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:

				if(configInfo.buttonStateCount[js.number] > 0) {  // Check if the button is a toggle or not

					if(js.value == 1) {  // If it is a multi-state and this event is a button press, cycle through states

						if(joystickState->button[js.number] < configInfo.buttonStateCount[js.number]) {

							joystickState->button[js.number] = joystickState->button[js.number] + 1;	

						} else {

							joystickState->button[js.number] = 0;  // Max state hit, zero out state

						}

					}

				} else {

					joystickState->button[js.number] = js.value;  // Normal button, switch to value from event

				}


				break;
			case JS_EVENT_AXIS:

				if(abs(js.value) > configInfo.jsDiscardUnder) {  // If the value is greater than our discard value set it
			
					jsValue = js.value;
					
				} else {

					jsValue = 0;  // Disregard values less than configInfo.jsDiscardUnder to avoid excessively sensitive sticks

				}

				if(js.number == THROTTLE) { // Handle throttle axis

					jsValue = jsValue * -1;  // Invert the throttle axis

					if(jsValue > 0) {  // Check if the joystick is in the "up" or "down" section of the axis

						if(jsValue > joystickState->prevLeftThrottle) {

							joystickState->axis[js.number] = jsValue;  // Joystick is in the up section and has passed previous max throttles
							joystickState->prevLeftThrottle = jsValue;

						}

					} else {

						jsValue = jsValue + 32767;   // Since we inverted the axis, add the MAX_VAL to this
						if(jsValue < joystickState->prevLeftThrottle) {  // Joystick is in the down section and has passed the previous min throttle

							joystickState->axis[js.number] = jsValue;
							joystickState->prevLeftThrottle = jsValue;

						}

					}
					
				} else {

					joystickState->axis[js.number] = jsValue;  // Regular axis, just store the current value

				}
				break;

			}

		}

}

/* translateJStoAF() - Translate current joystick settings to airframe settings (e.g. axis -> servos) */

void translateJStoAF(jsState joystickState) {

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

void sendCtrlUpdate(int signum) {
	
	int x;

	if(handShook) {	// We're synced up, send a control update

		int msgSize = MSG_HEADER_SIZE + SERVO_COUNT + 3 + 1; // MSG_HEADER_SIZE + 1 byte per servo + 3 bytes buttons + 1 checksum byte
		int buttonOffset = MSG_HEADER_SIZE + SERVO_COUNT;
		unsigned char *xbeeMsg;

		xbeeMsg = calloc(msgSize, sizeof(char)); // Allocate memory for our message

		xbeeMsg[0] = MSG_BEGIN;      // First character of xbeeMsg is MSG_BEGIN
		xbeeMsg[1] = MSG_TYPE_CTRL;  // Message type = control
		xbeeMsg[2] = msgSize;  // Message length = MSG_SIZE_CTRL
		xbeeMsg[3] = generateChecksum(xbeeMsg, MSG_HEADER_SIZE - 1); // Generate and store checksum
		for(x = 0 ; x < SERVO_COUNT ; x++) {

			xbeeMsg[x + MSG_HEADER_SIZE] = (unsigned char)airframeState.servos[x];  // Next 8 bytes are servo states

		}

		for(x = 0 ; x < 3 ; x++) {  // Next 3 bytes are 12 buttons, 2 bits per button

			xbeeMsg[x + buttonOffset] = (airframeState.buttons[(x * 4)] & 3) << 6;                       // Mask away anything but the last 2 bits and then bitshift to the left
			xbeeMsg[x + buttonOffset] = xbeeMsg[x + buttonOffset] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
			xbeeMsg[x + buttonOffset] = xbeeMsg[x + buttonOffset] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
			xbeeMsg[x + buttonOffset] = xbeeMsg[x + buttonOffset] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

		}

		xbeeMsg[msgSize - 1] = generateChecksum(xbeeMsg, msgSize - 1); // Store our checksum as our last byte

		writePortMsg(xbeePort, "XBee", xbeeMsg, msgSize); // Write out message to XBee
		free(xbeeMsg); // Deallocate memory for xbeeMsg
		#if DEBUG_LEVEL == 1
		printf("CSLA: %3d/%3d\n", commandsSinceLastAck, debugCommandsPerAck);
		#endif
		commandsSinceLastAck++;
	
	} else {  // We aren't synced up, send sync msg

		// Random length
		unsigned char *handshakeMsg;
		int msgSize = 10 + (rand() % 20); // Handshake message is random size between 10 and 30

		handshakeMsg = calloc(msgSize, sizeof(char)); // Allocate memory for handshakeMsg

		handshakeMsg[0] = MSG_BEGIN;
		handshakeMsg[1] = MSG_TYPE_SYNC;
		handshakeMsg[2] = msgSize;
		handshakeMsg[3] = generateChecksum(handshakeMsg, MSG_HEADER_SIZE - 1);
		for(x = MSG_HEADER_SIZE ; x < msgSize ; x++) {

			handshakeMsg[x] = rand() % 254;  // Build the random data portion of the handshake message
		
		}

		handshakeMsg[msgSize - 1] = generateChecksum(handshakeMsg, msgSize - 1); // Store our checksum as the last byte

		#if DEBUG_LEVEL == 0
		printf(".");
		#endif
		fflush(stdout);
		writePortMsg(xbeePort, "XBee", handshakeMsg, msgSize);  // Write the handshake to the XBee port
		free(handshakeMsg); // De-allocate memory for handshakeMsg

	}
}

/* writePortMsg(outputPort, portName, message, messageSize) - Write message to outputPort, deliver error if message fails to write */

void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize) {

	int msgWrote = 0;
	msgWrote = write(outputPort, message, messageSize);  // write() and store written byte count in msgWrote
	if(msgWrote != messageSize) { // If written byte count is not expected value
				
		printf("error writing to %s, wrote: %d/%d bytes.\n", portName, msgWrote, messageSize);  // Output error and info on what happened

	}


}

/* printState(axes) - Debug function, prints the current joystick and airframe states */

void printState(jsState joystickState) {

	int i;
	printf("\r");

	if(joystickState.axes) {
	printf("Axes: ");
	for (i = 0; i < joystickState.axes ; i++)
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

int readConfig(configValues *configInfo) {

	FILE *fp;
	int readCount = 0, lineBuffer = 1024;
	char line[lineBuffer];
	if ((fp = fopen(CONFIG_FILE, "r")) == NULL) {  // Open the config file read-only
		
		return -1;  // Return -1 if error opening

	} else {

		while (fgetsNoNewline(line, lineBuffer, fp) != NULL) {  // Read the entire file checking it line by line

			if(strcmp(line, "[XBee Port File]") == 0) {  // Line matches a config variable header, read the next line (value) and store it
		
				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {
					
					configInfo->xbeePortFile = calloc(strlen(line) + 1, sizeof(char)); // Allocate memory for our variable
					strcpy(configInfo->xbeePortFile, line);                            // Copy value into our var
					readCount++;                                                       // Increment our value count

				}

			} else if(strcmp(line, "[PPZ Port File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->ppzPortFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(configInfo->ppzPortFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Port File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->joystickPortFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(configInfo->joystickPortFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Event File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->joystickEventFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(configInfo->joystickEventFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Discard Threshold]") ==0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->jsDiscardUnder = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[PPM Interval]") ==0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->ppmInterval = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Commands Per Ack]") ==0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->commandsPerAck = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Button State Count]") ==0) {


				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					int bufferPos, currButton, strPos, buttonCount = 0;
					char buttonBuffer[4]; // More than 3 digits of a button state is a bit ridiculous
					int lineLength = strlen(line);
					for(strPos = 0 ; strPos < lineLength ; strPos++) { // Iterate through line[] character by character to get the button count

						if(line[strPos] == ',') { // Each comma separates a button state count so totalling them should get us our button count

							buttonCount++;

						}
					}

					configInfo->buttonStateCount = calloc(buttonCount, sizeof(int)); // Allocate memory for our button state counts

					currButton = 0;
					strPos = 0;

					while(currButton < buttonCount && strPos < lineLength) {  // Keep reading button states while we have lineLength left and buttonCount to meet

						for(bufferPos = 0 ; bufferPos < 4 ; bufferPos++) {

							buttonBuffer[bufferPos] = '\0'; // Null out buttonBuffer

						}

						bufferPos = 0;

						while(line[strPos] != ',') {

							buttonBuffer[bufferPos] = line[strPos];
							bufferPos++;
							strPos++;

						}

						strPos++; // Increment past the last comma

						configInfo->buttonStateCount[currButton] = atoi(buttonBuffer);

					}
					
				}
	
			}

		}

	}

	fclose(fp);

	printf("\nUsing config from %s:\n", CONFIG_FILE); // Print config values
	printf("                  XBee Port: %s\n", configInfo->xbeePortFile);
	printf("                   PPZ Port: %s\n", configInfo->ppzPortFile);
	printf("              Joystick Port: %s\n", configInfo->joystickPortFile);
	printf(" Joystick Discard Threshold: %7d\n", configInfo->jsDiscardUnder);
	printf("               PPM Interval: %7d\n", configInfo->ppmInterval);
	printf("           Commands Per Ack: %7d\n", configInfo->commandsPerAck);
	printf("         Button State Count: \n\n");

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

			*newLine = '\0';  // Replace annoying newline character with null Terminator (T-800)

		}
		return s;

	} else {

		return NULL;

	}

}

/* checkMessages() - Check for and handle any incoming messages */

int checkMessages(int msgPort, messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(msg->readBytes == MSG_HEADER_SIZE) {

		int x;	
		// Finished reading the message header, check it
		if(testChecksum(msg->messageBuffer, msg->readBytes)) { // Checksum was good

	
			msg->length = msg->messageBuffer[2];  // 0 - MSG_BEGIN, 1 - MSG_TYPE, 2 - MSG_LENGTH


		} else {			

			for(x = 0 ; x < (MSG_HEADER_SIZE - 1) ; x++) { // Shift all message characters to the left, drop the first one

				msg->messageBuffer[x] = msg->messageBuffer[x + 1];

			}

			msg->readBytes--; // Decrement byte count and chuck the first byte, this will allow us to reprocess the other 3 bytes in case we are desynched with the message sender

		}

	} 

	if(msg->readBytes < msg->length) { // Message is not finished being read

		if(read(xbeePort, &testByte, 1) == 1) {  // We read a byte, so process it

			#if DEBUG_LEVEL == 1
			printf("BYTE[%3d/%3d - HS:%d - CSLA: %3d]: %2x", msg->readMsgBytes, msg->msgWaitingBytes, handShook, commandsSinceLastAck, testByte); // Print out each received byte	
			#endif		

			msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
			msg->readBytes++;			  // Increment readBytes

			return 1;

		} else {

 			return 0;

		}

	} else { // Message is finished, process it

		if(testChecksum(msg->messageBuffer, msg->length)) { // Checksum passed, process message..  If the checksum failed we can assume corruption elsewhere since the header was legit

			processMessage(msg->messageBuffer, msg->length);

		} 

		int x;	

		// Clear out message so it's ready to be used again	
		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

			msg->messageBuffer[x] = '\0';

		}
		msg->readBytes = 0;            // Zero out readBytes
		msg->length = MSG_HEADER_SIZE; // Set message length to header length
		
		return 1;

	}

}

/* testChecksum(message, length) - Test if the last byte checksum is good */

int testChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 1
	printf("CHKMSG[%2d]: ", length);
	for(x = 0 ; x < length ; x++) {

		printf("%2x ", (unsigned int)message[x]); // Print the whole message in hex

	}
	printf("CHK: "); // Print the checksum marker
	#endif	

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x];  // Test the message against its checksum (last byte)

	}

	#if DEBUG_LEVEL == 1
	printf("%2x\n\n", checksum); // Print the checksum
	#endif	

	if(checksum == 0x00) {

		return 1;  // Checksum passed!

	} else {

		return 0;

	}

}

/* generateChecksum(message, length) - Generate a checksum for message */

unsigned char generateChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x]; // Generate checksum

	}

	return checksum;

}

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(unsigned char *message, int length) {

	unsigned char msgType = message[1];

	if(msgType == MSG_TYPE_SYNC) {  // Handle the message, since it got past checksum it has to be legit

		handShook = 1;
		commandsSinceLastAck = 0;
		#if DEBUG_LEVEL == 1
		printf("CSLA: %3d/%3d\n", commandsSinceLastAck, debugCommandsPerAck);
		#endif

	} else if(msgType == MSG_TYPE_CTRL) { // We shouldn't get this from the Arduino
	} else if(msgType == MSG_TYPE_PPZ) { // Handle PPZ message
	} else if(msgType == MSG_TYPE_CFG) { // This either
	}

}

/* checkSignal() - Check whether the signal is good, if not RUMBLE!! */

void checkSignal(int commandsPerAck) {

	if(commandsSinceLastAck > commandsPerAck) {  // Looks like we've lost our signal (2s and 100+ cmds since last ACK)

		handShook = 0;  // Turn off handshake so we can resync
		printf("Lost signal!  Attempting to resync.\n");
		#if DEBUG_LEVEL == 1
		printf("CSLA: %3d\n", commandsSinceLastAck);
		#endif

	}

	if(commandsSinceLastAck > 120) {  // Small rumble
	} else if(commandsSinceLastAck > 500) {  // 10s!  Big rumble!
	}

}
