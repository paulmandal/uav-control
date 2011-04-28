/* js_ctrl.c - Paul Mandal (paul.mandal@gmail.com)
 *
 * Tracks joystick updates
 * Sends JS state every 20ms
 * Receives commands from PPZ, relays to UAV
 * Receives telemetry from UAV, relays to PPZ
 *
 * Special thanks to Vojtech Pavlik <vojtech@ucw.cz>, I adopted much of the joystick code from jstest.c
 * Special thanks to Johann Deneux <deneux@ifrance.com>, I learned the Force Feedback methods from fftest.c
*/

/*

TODO:

- DPad -> servo control
- Force feedback on RSSI weak/loss

Adruino:

- Handle config updates via msg
- Handle digital pins/buttons
- Handle 3-way switch (or servo it?)

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
#include <stdint.h>
#include <termios.h>
#include <time.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "axbtnmap.h"

#include <signal.h>

/* Definitions */

#define DEBUG_LEVEL 1	      // Debug level - tells compiler to include or exclude debug message code
			      // Debug level - 1 - Lost signal debug
			      // Debug level - 2 - Debug joystick position info
			      // Debug level - 3 - Debug incoming messages
			      // Debug level - 4 - Time incoming messages

#define VERSION_MAJOR 2       // Version information, Major #
#define VERSION_MINOR 9       // Minor #
#define VERSION_MOD   7       // Mod #
#define VERSION_TAG   "DBG"   // Tag

#define MSG_BEGIN     0xFF    // Begin of control message indicator byte
#define MSG_TYPE_CTRL 0x01    // Control update message type indicator
#define MSG_TYPE_CFG  0x02    // Configuration update
#define MSG_TYPE_PPZ  0x03    // Message from PPZ
#define MSG_TYPE_DBG  0x04    // Debug message
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

typedef struct _ptyInfo {

	int master;
	char *slaveDevice;

} ptyInfo;

/* Let's do sum prototypes! */

int openPort(char *portName, char *use);
int openPty(ptyInfo *pty, char *use);
int openJoystick(char *portName, jsState *joystickState);
int readConfig(configValues *configInfo);
void initTimer(configValues configInfo);
void initAirframe();
void translateJStoAF(jsState joystickState);
void readJoystick(int jsPort, jsState *joystickState, configValues configInfo);
int initMessage(messageState *message);
int checkXBeeMessages(int msgPort, messageState *msg);
int checkPPZMessages(int msgPort, messageState *msg);
void processMessage(messageState *msg);
void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize);
unsigned char generateChecksum(unsigned char *message, int length);
int testChecksum(unsigned char *message, int length);
void sendCtrlUpdate (int signum);
int checkSignal(int commandsPerAck);
void printState(jsState joystickState);
int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh);
char *fgetsNoNewline(char *s, int n, FILE *stream);

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
ptyInfo ppzPty;
int xbeePort;          // XBee port FD

int commandsSinceLastAck = 0;  // Commands sent since last ACK
int handShook = 0;             // Handshook?

unsigned long totalMsgs = 0;

#if DEBUG_LEVEL > 0
int debugCommandsPerAck = 0;
#endif

time_t startTime;

/* Main function */

int main(int argc, char **argv)
{

	startTime = time(NULL);
	int jsPort;                // JoystickPort FD
	jsState joystickState;     // Current joystick state
	configValues configInfo;   // Configuration values
	messageState xbeeMsg;	   // messageState for incoming XBee message
	messageState ppzMsg;	   // messageState for incoming PPZ message

	initMessage(&xbeeMsg);
	initMessage(&ppzMsg);
	ppzMsg.readBytes = MSG_HEADER_SIZE; // Leave space for the addition of a header to the msg from GCS
	
	printf("Starting js_crl version %d.%d.%d-%s...\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD, VERSION_TAG);  // Print version information

	srand(time(NULL));  // Init random using current time
	if(readConfig(&configInfo) < 0) { // Read our config into our config vars

		perror("js_ctrl"); // Error reading config file
		return 1;

	}

	initAirframe();  // Init airframe state

	if((xbeePort = openPort(configInfo.xbeePortFile, "XBee")) < 0) { // open the XBee port
		return 1;
	}

	if(openPty(&ppzPty, "PPZ") < 0) { // open the PPZ pty

		return 1;	
		
	} 
	
	if((jsPort = openJoystick(configInfo.joystickPortFile, &joystickState)) < 0) { // open the Joystick
		return 1;
	}

	initTimer(configInfo); 	// Set up timer (every 20ms)
	printf("\nPPZ pty file is: %s\n\n", ppzPty.slaveDevice);
	printf("Ready to read JS & relay for PPZ...\n\n");

	while(1) {

		if(!handShook) {

			#if DEBUG_LEVEL != 4
			printf("Handshaking..");
			#endif
			#if DEBUG_LEVEL == 1	
			printf("\n");
			#endif

			while(!handShook) {  // Handshaking loop

				if(!checkXBeeMessages(xbeePort, &xbeeMsg)) { // Check for pending msg bytes

					usleep(100); // If nothing is there pause for 100usec, handshake is sent out by interrupt at 50Hz

				} else {
				
					usleep(10); // Give 10usec for character to be removed from buffer by read()
				
				}

			}

			#if DEBUG_LEVEL != 4
			printf("got ACK, handshake complete!\n\n");
			#endif
	
		}

		readJoystick(jsPort, &joystickState, configInfo);  // Check joystick for updates

		translateJStoAF(joystickState);	// update Airframe model

		#if DEBUG_LEVEL == 2
		printState(joystickState); 	// print JS & AF state
		#endif
		
		int x;
		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {  // Try to read MSG_BUFFER_SIZE bytes per loop

			checkXBeeMessages(xbeePort, &xbeeMsg); // Check for pending msg bytes
			checkPPZMessages(ppzPty.master, &ppzMsg); // Check for pending msg bytes
			usleep(10); // Pause for 10usec

		}

		checkSignal(configInfo.commandsPerAck);  // Check if our signal is still good
	
	}

	free(xbeeMsg.messageBuffer); // Meh
	free(ppzMsg.messageBuffer); // Not that this matters..
	return 0;

}

/* Function definitions */

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

/* openPty() - Open a pty for communication with the PPZ GCS software */

int openPty(ptyInfo *pty, char *use) {

	printf("Opening pty for %s..\n", use);
	if((pty->master = posix_openpt(O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {  // Create our pty with posix_openpt()
	
		perror("js_ctrl");
		return 0;
	
	}
	
	if((grantpt(pty->master) == -1) || (unlockpt(pty->master) == -1) || ((pty->slaveDevice = ptsname(pty->master)) == NULL)) { // Grant permissions and unlock our pty, then return the device name for display to the user
	
		perror("js_ctrl");
		return 0;
	
	}
	
	struct termios options;  // The port opened, set it up the port

	tcgetattr(pty->master, &options);              // Get current settings
	cfsetispeed(&options, B115200);        // Set input speed to 115200
	cfsetospeed(&options, B115200);        // Set output speed to 115200
	options.c_cflag |= (CLOCAL | CREAD);  // Set sum flags (CLOCAL & CREAD)
	options.c_lflag &= (~ECHO); // Turn local echo off
	tcsetattr(pty->master, TCSANOW, &options);     // Set options
	
	return 1;

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

/* readConfig() - Read values from configuration file */

int readConfig(configValues *configInfo) {

	FILE *fp;
	int x, buttonCount = 1, readCount = 0, lineBuffer = 1024;
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

					int bufferPos, currButton, strPos;
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

						while(line[strPos] != ',' && line[strPos] != '\0' && strPos < lineLength) {

							buttonBuffer[bufferPos] = line[strPos];
							bufferPos++;
							strPos++;

						}

						strPos++; // Increment past the last comma
						configInfo->buttonStateCount[currButton] = atoi(buttonBuffer);
						currButton++;

					}
					
				}

			}

		}

	}

	fclose(fp);

	printf("\nUsing config from %s:\n", CONFIG_FILE); // Print config values
	printf("                  XBee Port: %s\n", configInfo->xbeePortFile);
	printf("              Joystick Port: %s\n", configInfo->joystickPortFile);
	printf(" Joystick Discard Threshold: %7d\n", configInfo->jsDiscardUnder);
	printf("               PPM Interval: %7d\n", configInfo->ppmInterval);
	printf("           Commands Per Ack: %7d\n", configInfo->commandsPerAck);
	printf("         Button State Count: \n\n");
	
	for(x = 0 ; x < buttonCount ; x++) {
	
		if(x != 0 && x % 4 == 0) {
			printf("\n");
		}
		printf("%2d: %2d		", x + 1, configInfo->buttonStateCount[x]);
	
	}
	printf("\n\n");

	if(readCount >= CONFIG_FILE_MIN_COUNT) {

		return 1;  // Read enough config variables, success

	} else {

		return 0;  // Didn't read enough of config values

	}	

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

/* initAirframe() - Initalise airframe state */

void initAirframe() {

	int i;

	for(i = 0 ; i < 8 ; i++) {

		airframeState.servos[i] = 0;  // Set all servo pos to 0

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

/* initMessage() - Initialise message */

int initMessage(messageState *message) {

	message->readBytes = 0;
	message->length = MSG_HEADER_SIZE; // Init message.length as header length size

	if((message->messageBuffer = calloc(MSG_BUFFER_SIZE, sizeof(char))) != NULL) {
	
		return 1; // calloc() worked
		
	} else {
	
		return 0; // calloc() failed
	
	}	
	
}

/* checkXBeeMessages() - Check for and handle any incoming messages */

int checkXBeeMessages(int msgPort, messageState *msg) {

	unsigned char testByte = 0x00;
	int msgGood = 0;
	
	if(msg->readBytes == MSG_HEADER_SIZE) {

		int x;	
		// Finished reading the message header, check it

		// First character must be a MSG_BEGIN
		
		if(msg->messageBuffer[0] == MSG_BEGIN) {
		
			if(testChecksum(msg->messageBuffer, msg->readBytes)) { // Checksum was good

	
				msg->length = msg->messageBuffer[2];  // 0 - MSG_BEGIN, 1 - MSG_TYPE, 2 - MSG_LENGTH
				msgGood = 1;


			} 			
			
		} 
		
		if(!msgGood) { // Something was wrong with the message header
		
		
			for(x = 0 ; x < (MSG_HEADER_SIZE - 1) ; x++) { // Shift all message characters to the left, drop the first one

				msg->messageBuffer[x] = msg->messageBuffer[x + 1];
		
			}

			msg->readBytes--; // Decrement byte count and chuck the first byte, this will allow us to reprocess the other 3 bytes in case we are desynched with the message sender
		
		}

	} 

	if(msg->readBytes < msg->length) { // Message is not finished being read

		if(read(msgPort, &testByte, sizeof(char)) == sizeof(char)) {  // We read a byte, so process it

			#if DEBUG_LEVEL == 3
			printf("BYTE[%3d/%3d - HS:%d - CSLA: %3d]: %2x\n", msg->readBytes, msg->length, handShook, commandsSinceLastAck, testByte); // Print out each received byte	
			#endif		

			msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
			msg->readBytes++;			  // Increment readBytes

			return 1;

		} else {

 			return 0;

		}

	} else { // Message is finished, process it

		if(testChecksum(msg->messageBuffer, msg->length)) { // Checksum passed, process message..  If the checksum failed we can assume corruption elsewhere since the header was legit

			processMessage(msg);

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

/* checkPPZMessages() - Check for and handle any incoming PPZ messages */

int checkPPZMessages(int msgPort, messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(read(msgPort, &testByte, sizeof(char)) == sizeof(char)) {

		msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
		msg->readBytes++;			       // Increment readBytes
                msg->length++;                                 // Increment length
		
		if(testByte == '\n') { // This is the message end, relay the message to UAV and reset msg
		
			msg->messageBuffer[0] = MSG_BEGIN;
			msg->messageBuffer[1] = MSG_TYPE_PPZ;
			msg->messageBuffer[2] = msg->length;
			msg->messageBuffer[3] = generateChecksum(msg->messageBuffer, MSG_HEADER_SIZE);
			msg->messageBuffer[msg->length - 1] = generateChecksum(msg->messageBuffer, msg->length - 1);
			writePortMsg(xbeePort, "XBee", msg->messageBuffer, msg->length);

			msg->readBytes = MSG_HEADER_SIZE;  // Leave room for header to be added
                        msg->length = MSG_HEADER_SIZE;
			int x;	

			// Clear out message so it's ready to be used again	
			for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

				msg->messageBuffer[x] = '\0';

			}
		
		}
	
		return 1;
	
	} else {

		return 0;
	
	}

}

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(messageState *msg) {

	unsigned char msgType = msg->messageBuffer[1];
	int x;

	if(msgType == MSG_TYPE_SYNC) {  // Handle the message, since it got past checksum it has to be legit

		handShook = 1;
		commandsSinceLastAck = 0;
		#if DEBUG_LEVEL == 3
		printf("Got sync msg, CSLA: %3d/%3d\n", commandsSinceLastAck, debugCommandsPerAck);
		#endif

	} else if(msgType == MSG_TYPE_CTRL) { // We shouldn't get this from the Arduino
	} else if(msgType == MSG_TYPE_PPZ) { // Handle PPZ message
	
		msg->messageBuffer[msg->length - 1] = '\0'; // Replace checksum with null
		for(x = 0 ; x < msg->length - MSG_HEADER_SIZE ; x++) {
		
			msg->messageBuffer[x] = msg->messageBuffer[x + MSG_HEADER_SIZE];  // Shift everything to the left MSG_HEADER_SIZE bytes
		
		}
		
		writePortMsg(ppzPty.master, "PPZ", msg->messageBuffer, msg->length - MSG_HEADER_SIZE); // Write out the message, minus the header size
	
	} else if(msgType == MSG_TYPE_CFG) { // This either
	} else if(msgType == MSG_TYPE_DBG) { // This is a debug message, print it
	
		#if DEBUG_LEVEL == 4
		time_t currentTime = time(NULL);
		time_t diff = currentTime - startTime;
		printf("DEBUG MSG from UAV [%lds since last]: ", diff);
		startTime = currentTime;
		#else
		printf("DEBUG MSG from UAV: ");
		#endif

		for(x = MSG_HEADER_SIZE ; x < msg->length - 1 ; x++) {
		
		
			printf("%c", msg->messageBuffer[x]);
		
		}
		printf("\n");
	}

}

/* writePortMsg(outputPort, portName, message, messageSize) - Write message to outputPort, deliver error if message fails to write */

void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize) {

	int msgWrote = 0;
	#if DEBUG_LEVEL == 21
	printf("WRITING[%2d]: ", messageSize);
	for(x = 0 ; x < messageSize ; x++) {
		
		printf("%2x ", message[x]);
	
	}
	printf("\n");
	#endif
	msgWrote = write(outputPort, message, messageSize);  // write() and store written byte count in msgWrote
	if(msgWrote != messageSize) { // If written byte count is not expected value
				
		printf("error writing to %s, wrote: %d/%d bytes.\n", portName, msgWrote, messageSize);  // Output error and info on what happened

	}

}

/* generateChecksum(message, length) - Generate a checksum for message */

unsigned char generateChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 21
	printf("GENCHK[%2d]: ", length);
	for(x = 0 ; x < length ; x++) {

		printf("%2x ", (unsigned int)message[x]); // Print the whole message in hex

	}
	printf("CHK: "); // Print the checksum marker
	#endif	

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x]; // Generate checksum

	}

	#if DEBUG_LEVEL == 21
	printf("%2x\n\n", checksum); // Print the checksum
	#endif	

	return checksum;

}

/* testChecksum(message, length) - Test if the last byte checksum is good */

int testChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 3
	printf("CHKMSG[%2d]: ", length);
	for(x = 0 ; x < length ; x++) {

		printf("%2x ", (unsigned int)message[x]); // Print the whole message in hex

	}
	printf("CHK: "); // Print the checksum marker
	#endif	

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x];  // Test the message against its checksum (last byte)

	}

	#if DEBUG_LEVEL == 3
	printf("%2x\n\n", checksum); // Print the checksum
	#endif	

	if(checksum == 0x00) {

		return 1;  // Checksum passed!

	} else {

		return 0;

	}

}

/* sendCtrlUpdate() - Send latest control state to XBee port */

void sendCtrlUpdate(int signum) {
	
	int x;

	if(handShook) {	// We're synced up, send a control update

		int msgSize = MSG_HEADER_SIZE + SERVO_COUNT + 3 + 1; // MSG_HEADER_SIZE + 1 byte per servo + 3 bytes buttons + 1 checksum byte
		int buttonOffset = MSG_HEADER_SIZE + SERVO_COUNT;
		unsigned char *ctrlMsg;

		ctrlMsg = calloc(msgSize, sizeof(char)); // Allocate memory for our message

		ctrlMsg[0] = MSG_BEGIN;      // First character of ctrlMsg is MSG_BEGIN
		ctrlMsg[1] = MSG_TYPE_CTRL;  // Message type = control
		ctrlMsg[2] = msgSize;  // Message length = MSG_SIZE_CTRL
		ctrlMsg[3] = generateChecksum(ctrlMsg, MSG_HEADER_SIZE - 1); // Generate and store checksum
		for(x = 0 ; x < SERVO_COUNT ; x++) {

			ctrlMsg[x + MSG_HEADER_SIZE] = (unsigned char)airframeState.servos[x];  // Next 8 bytes are servo states

		}

		for(x = 0 ; x < 3 ; x++) {  // Next 3 bytes are 12 buttons, 2 bits per button

			ctrlMsg[x + buttonOffset] = (airframeState.buttons[(x * 4)] & 3) << 6;                       // Mask away anything but the last 2 bits and then bitshift to the left
			ctrlMsg[x + buttonOffset] = ctrlMsg[x + buttonOffset] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
			ctrlMsg[x + buttonOffset] = ctrlMsg[x + buttonOffset] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
			ctrlMsg[x + buttonOffset] = ctrlMsg[x + buttonOffset] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

		}

		ctrlMsg[msgSize - 1] = generateChecksum(ctrlMsg, msgSize - 1); // Store our checksum as our last byte

		writePortMsg(xbeePort, "XBee", ctrlMsg, msgSize); // Write out message to XBee
		free(ctrlMsg); // Deallocate memory for ctrlMsg
		totalMsgs++;
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

			handshakeMsg[x] = (unsigned char)(rand() % 254);  // Build the random data portion of the handshake message
		
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

/* checkSignal() - Check whether the signal is good, if not RUMBLE!! */

int checkSignal(int commandsPerAck) {

	struct ff_effect effects[2]; // 2 ff_effect structs, for weak and strong rumble
	struct input_event play, stop; // input_event control to play and stop effects

	if(commandsSinceLastAck > commandsPerAck) {  // Looks like we've lost our signal (2s and 100+ cmds since last ACK)

		handShook = 0;  // Turn off handshake so we can resync
		#if DEBUG_LEVEL != 4
		printf("Lost signal!  Attempting to resync.\n");
		#endif
		#if DEBUG_LEVEL == 1
		time_t currentTime = time(NULL);
		time_t diff = currentTime - startTime;
		printf("CSLA: %3d Total msgs: %lu Running: %lds\n", commandsSinceLastAck, totalMsgs, diff);
		#endif
		return 0;

	}

	if(commandsSinceLastAck > 120 && commandsSinceLastAck < 501) {  // Small rumble
	
		return 0;
	
	} else if(commandsSinceLastAck > 500) {  // 10s!  Big rumble!
	
		return 0;
	
	} else {
	
		return 1;
	
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

/* map() - Map a number in inRangeLow->inRangeHigh range into outRangeLow->outRangeHigh */

int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh)
{
	return outRangeLow + (value-inRangeLow)*(outRangeHigh-outRangeLow)/(inRangeHigh-inRangeLow);
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
