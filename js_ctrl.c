/* js_ctrl.c - Paul Mandal (paul.mandal@gmail.com)
 *
 * Tracks joystick updates
 * Sends JS state every 20ms
 * Receives commands from PPZ, relays to UAV
 * Receives telemetry from UAV, relays to PPZ
 * Rumble on weak RSSI or signal loss
 *
 * Special thanks to Vojtech Pavlik <vojtech@ucw.cz>, I adopted much of the joystick code from jstest.c
 * Special thanks to Johann Deneux <deneux@ifrance.com>, I learned the Force Feedback methods from fftest.c
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
#include <math.h>

#include <linux/input.h>
#include <linux/joystick.h>

#include "axbtnmap.h"

#include <signal.h>

/* Definitions */

#define DEBUG_LEVEL 0	      // Debug level - tells compiler to include or exclude debug message code
			      // Debug level - 1 - Lost signal debug
			      // Debug level - 2 - Debug joystick position info
			      // Debug level - 3 - Debug incoming messages
			      // Debug level - 4 - Time incoming messages
			      // Debug level - 5 - Bad checksum reports
			      // Debug level - 6 - Debug outgoing messages
			      // Debug level - 7 - Debug servo encoding

#define VERSION_MAJOR 3       // Version information, Major #
#define VERSION_MINOR 2       // Minor #
#define VERSION_MOD   1       // Mod #
#define VERSION_TAG   "DBG"   // Tag

#define MSG_BEGIN            0xFF    // Begin of control message indicator byte
#define MTYPE_PING           0x00    // Message types: Ping
#define MTYPE_PING_REPLY     0x01    // Ping reply
#define MTYPE_HEARTBEAT      0x02    // Heartbeat (no control update)
#define MTYPE_FULL_UPDATE    0x03    // Full update
#define MTYPE_SINGLE_SERVO   0x04    // Single servo
#define MTYPE_VAR_SERVOS     0x05    // Variable # of servos
#define MTYPE_ALL_SERVOS     0x06    // Update all servos
#define MTYPE_BUTTON_UPDATE  0x08    // Buttons update
#define MTYPE_PPZ            0x09    // PPZ Message
#define MTYPE_DEBUG          0x10    // Debug message
#define MTYPE_RESET          0x11    // Reset component (e.g. XBee, GPS)
#define MTYPE_STATUS	     0x12    // Status message

#define MTYPE_PING_SZ            4    // Message types: Ping
#define MTYPE_PING_REPLY_SZ      4    // Ping reply
#define MTYPE_HEARTBEAT_SZ       3    // Heartbeat (no control update)
#define MTYPE_FULL_UPDATE_SZ    22    // Full update
#define MTYPE_SINGLE_SERVO_SZ    5    // Single servo
#define MTYPE_ALL_SERVOS_SZ     19    // Update all servos
#define MTYPE_BUTTON_UPDATE_SZ   6    // Buttons update
#define MTYPE_STATUS_SZ		 5    // Status message

#define MAX_CTRL_MSG_SZ         19    // Maximum data size for a control update msg

#define MSG_BUFFER_SIZE     256      // Message buffer size in bytes
#define PPZ_MSG_HEADER_SIZE   3      // PPZ msg header size in bytes

#define NAME_LENGTH 128       // Length of Joystick name
#define CAM_PAN 4	      // Camera Pan axis #
#define CAM_PAN_SRC 3         // Camera Pan axis source
#define CAM_TILT 5            // Camera Tilt axis #
#define CAM_TILT_SRC 2        // Camera Tilt axis source
#define ROLL 3                // Roll axis #
#define PITCH 2               // Pitch axis #
#define YAW 0                 // Yaw axis #
#define THROTTLE 1            // Throttle axis #

#define SERVO_COUNT   8       // Total servos on airframe
#define BUTTON_COUNT 12       // Buttons on joystick

#define PPZ_MSG_SIZE_CTRL 1024

#define SRV_THROTTLE 0
#define SRV_LEFTWING 1
#define SRV_RIGHTWING 2
#define SRV_L_ELEVRON 3
#define SRV_R_ELEVRON 4
#define SRV_CAM_PAN 5
#define SRV_CAM_TILT 6
#define SRV_LANDING 7

#define CONFIG_FILE "js_ctrl.rc"  // Config file name
#define CONFIG_FILE_MIN_COUNT 8   // # of variables stored in config file 

#define EFFECTS_COUNT 16

double THROTTLE_SAFETY_DEBUG = 10.0;

/* Structures */

typedef struct _jsState {  // Store the axis and button states
	int port;
	int event;
	int *axis;
	char *button;
	int rumbleLevel;
	time_t lastRumbleTime;	
	unsigned char axes;
	unsigned char buttons;
	struct ff_effect effects[EFFECTS_COUNT];
	
} jsState;

typedef struct _afState { // Store the translated (servo + buttons) states, globally accessible

	unsigned int servos[SERVO_COUNT];
	unsigned int buttons[BUTTON_COUNT];
	int prevThrottle;
	int servosChanged[SERVO_COUNT];
	int buttonsChanged[BUTTON_COUNT];
	int batteryVoltage;

} afState;

typedef struct _configValues {

	char *joystickEventFile; // joystickEvent filename (for RUMBLE!)
	char *joystickPortFile;  // joystickPort filename
	char *xbeePortFile;      // xbeePort filename
	int *buttonStateCount;   // Button state counts
	int jsDiscardUnder;	 // Joystick discard under threshold
	int ppmInterval;	 // Interval to send commands to XBee
	double timeoutThreshold; // How long without a message before timeout
	int contextButton;       // Button that affects joystick context

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

typedef struct _portState {

	ptyInfo ppzPty;	       // Pseudoterminal for PPZ comms
	int xbeePort;          // XBee port FD

} portState;

typedef struct _signalState {

	time_t lastMessageTime;    // Time of last message reply
	int handShook;             // Handshook?
	int localRSSI;		   // Local RSSI
	int remoteRSSI;		   // Remote RSSI
	int videoRSSI;		   // Video downlink RSSI
	unsigned char pingData;    // Random character sent along with last ping
	int ctrlCounter;           // Counter for outbound control messages
	time_t lostSignalTime;     // Time signal was lost
	unsigned long totalMsgs;

} signalState;

/* Let's do sum prototypes! */

int openPort(char *portName, char *use);
int openPty(ptyInfo *pty, char *use);
int openJoystick(configValues configInfo, jsState *joystickState);
int readConfig(configValues *configInfo);
void initGlobals();
void initTimer(configValues configInfo);
void initAirframe();
void initRumble(jsState *joystickState);
void translateJStoAF(jsState joystickState, configValues configInfo);
void readJoystick(jsState *joystickState, configValues configInfo);
int initMessage(messageState *message);
int checkXBeeMessages(int msgPort, messageState *msg);
int checkPPZMessages(int msgPort, messageState *msg);
void processMessage(messageState *msg);
int getMessageLength(messageState *msg);
void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize);
unsigned char generateChecksum(unsigned char *message, int length);
int testChecksum(unsigned char *message, int length);
void sendCtrlUpdate (int signum);
int checkSignal(configValues configInfo, jsState *joystickState);
void sendAck(messageState *msg);
void printState(jsState joystickState);
int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh);
char *fgetsNoNewline(char *s, int n, FILE *stream);
void printOutput();

/* Global state storage variables */

// global variables to store states

afState airframeState; // Current airframe state
portState ports;
signalState signalInfo;

#if DEBUG_LEVEL > 0
int debugCommandsPerAck = 0;
#endif

time_t startTime;

/* Main function */

int main(int argc, char **argv)
{

	startTime = time(NULL);
	jsState joystickState;     // Current joystick state
	configValues configInfo;   // Configuration values
	messageState xbeeMsg;	   // messageState for incoming XBee message
	messageState ppzMsg;	   // messageState for incoming PPZ message
	
	initGlobals();

	initMessage(&xbeeMsg);
	initMessage(&ppzMsg);
	
	ppzMsg.readBytes = PPZ_MSG_HEADER_SIZE; // Leave space for the addition of a header to the msg from GCS
	ppzMsg.length = PPZ_MSG_HEADER_SIZE;
	
	printf("Starting js_crl version %d.%d.%d-%s...\n", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD, VERSION_TAG);  // Print version information

	srand(time(NULL));  // Init random using current time
	if(readConfig(&configInfo) < 0) { // Read our config into our config vars

		perror("js_ctrl"); // Error reading config file
		return 1;

	}

	initAirframe();  // Init airframe state

	if((ports.xbeePort = openPort(configInfo.xbeePortFile, "XBee")) < 0) { // open the XBee port
		return 1;
	}

	if(openPty(&ports.ppzPty, "PPZ") < 0) { // open the PPZ pty

		return 1;	
		
	} 
	
	if(openJoystick(configInfo, &joystickState) < 0) { // open the Joystick
		return 1;
	}

	initRumble(&joystickState);   // Set up rumble effects
	initTimer(configInfo); 	// Set up timers
	printf("\nPPZ pty file is: %s\n\n", ports.ppzPty.slaveDevice);
	printf("Ready to read JS & relay for PPZ...\n\n");

	while(1) {

		if(!signalInfo.handShook) {

			#if DEBUG_LEVEL != 4 && DEBUG_LEVEL != 5
			printf("Handshaking..");
			#endif
			#if DEBUG_LEVEL == 1	
			printf("\n");
			#endif
			while(!signalInfo.handShook) {  // Handshaking loop

				checkXBeeMessages(ports.xbeePort, &xbeeMsg); // Check for pending msg bytes
				usleep(10); // Give 10usec for character to be removed from buffer by read()
				checkSignal(configInfo, &joystickState); // call checkSignal() to allow rumble

			}

			#if DEBUG_LEVEL != 4 && DEBUG_LEVEL != 5
			printf("got ACK, handshake complete!\n\n");
			#endif
	
		}
		
		int x;
		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {  // Try to read MSG_BUFFER_SIZE bytes per loop.. checkSignal() is the only thing we don't really need to do continually

			readJoystick(&joystickState, configInfo);  // Check joystick for updates
		
			translateJStoAF(joystickState, configInfo);	// update Airframe model

			#if DEBUG_LEVEL == 2
			printState(joystickState); 	// print JS & AF state
			#endif

			checkXBeeMessages(ports.xbeePort, &xbeeMsg); // Check for pending msg bytes
			//checkPPZMessages(ports.ppzPty.master, &ppzMsg); // Check for pending msg bytes
			usleep(10); // Pause for 10usec

		}

		checkSignal(configInfo, &joystickState);  // Check if our signal is still good
	
	}

	free(xbeeMsg.messageBuffer); // Meh
	free(ppzMsg.messageBuffer); // Not that this matters..
	return 0;

}

/* Function definitions */

/* initGlobals() - initalise all global vars */

void initGlobals() {

	signalInfo.handShook = 0;
	signalInfo.totalMsgs = 0;
	signalInfo.localRSSI = 0;
	signalInfo.remoteRSSI = 0;
	signalInfo.ctrlCounter = 0;
	signalInfo.lastMessageTime = time(NULL);
	signalInfo.lostSignalTime = time(NULL);
	
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
	cfsetispeed(&options, B115200);       // Set input speed to 115200
	cfsetospeed(&options, B115200);       // Set output speed to 115200
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

int openJoystick(configValues configInfo, jsState *joystickState) {

	uint16_t btnmap[BTNMAP_SIZE];
	uint8_t axmap[AXMAP_SIZE];
	int btnmapok = 1;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";
	int fd, efd, i;

	printf("Opening joystick %s..\n", configInfo.joystickPortFile);
	if ((fd = open(configInfo.joystickPortFile, O_RDWR | O_NONBLOCK)) < 0) {  // Open joystick port in non-blocking mode
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

	printf("Opening joystick event file %s..\n", configInfo.joystickEventFile);
	if ((efd = open(configInfo.joystickEventFile, O_RDWR)) < 0) {  // Open joystick port in non-blocking mode
		perror("js_ctrl");  // Error opening port
		return -1;
	}

	joystickState->rumbleLevel = 0;
	joystickState->lastRumbleTime = time(NULL);

	joystickState->port = fd;   // Store joystick Port file descriptor
	joystickState->event = efd; // Store joystick Event file descriptor
	return 0;

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

			} else if(strcmp(line, "[Joystick Discard Threshold]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->jsDiscardUnder = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[PPM Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->ppmInterval = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Context Button]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->contextButton = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Timeout Threshold]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo->timeoutThreshold = atof(line); // Translate ASCII -> double
					readCount++;

				}

			} else if(strcmp(line, "[Button State Count]") == 0) {

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
	printf(" Joystick Discard Threshold: %7d \n", configInfo->jsDiscardUnder);
	printf("               PPM Interval: %7d μs\n", configInfo->ppmInterval);
	printf("          Timeout Threshold: %7.2f s\n", configInfo->timeoutThreshold);
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

/* initTimer() - set up timer */

void initTimer(configValues configInfo) {
 
	struct sigaction saCtrl;
	struct itimerval timerCtrl;

	printf("Setting up pulse timer..\n");
	memset(&saCtrl, 0, sizeof (saCtrl));                       // Make signal object
	saCtrl.sa_handler = &sendCtrlUpdate;                   // Set signal function handler in 'saCtrl'
	sigaction(SIGALRM, &saCtrl, NULL);                     // Set SIGALRM signal handler

	timerCtrl.it_value.tv_sec = 0;
	timerCtrl.it_value.tv_usec = configInfo.ppmInterval;    // Set timer interval to 20000usec / 4 (5ms)
	timerCtrl.it_interval.tv_sec = 0;
	timerCtrl.it_interval.tv_usec = configInfo.ppmInterval; // Set timer reset to 20000usec / 4 (5ms)

	printf("Starting pulse timer..\n");
	setitimer(ITIMER_REAL, &timerCtrl, NULL);               // Start the timer


}

/* initAirframe() - Initalise airframe state */

void initAirframe() {

	int i;

	for(i = 0 ; i < SERVO_COUNT ; i++) {

		airframeState.servos[i] = 511;  // Set all servo pos to 511

	}

	airframeState.servos[THROTTLE] = 0;
	airframeState.batteryVoltage = 0;
	airframeState.prevThrottle = 0;

}

/* initRumble() - Initalise rumble effects */

void initRumble(jsState *joystickState) {

	int x;
	int strongMagnitude;
	int weakMagnitude;
	
	for(x = 0 ; x < EFFECTS_COUNT ; x++) {
		
		strongMagnitude = x < 3 ? 0 : (0xffff / 12) * (x - 3);
		weakMagnitude = (0xffff / 8) * (x + 1);
		joystickState->effects[x].type = FF_RUMBLE;
		joystickState->effects[x].id = -1;
		joystickState->effects[x].u.rumble.strong_magnitude = strongMagnitude;
		joystickState->effects[x].u.rumble.weak_magnitude = weakMagnitude > 0xffff ? 0xffff : weakMagnitude;
		joystickState->effects[x].replay.length = 1000;
		joystickState->effects[x].replay.delay = 0;
		
		if (ioctl(joystickState->event, EVIOCSFF, &joystickState->effects[x]) == -1) {
			perror("Upload effects[x]");
		}
	
	}

}

/* translateJStoAF() - Translate current joystick settings to airframe settings (e.g. axis -> servos) */

void translateJStoAF(jsState joystickState, configValues configInfo) {

	int x;
	x = map(joystickState.axis[ROLL], -32767, 32767, 0, 1023);
	if(x != airframeState.servos[ROLL]) {

		airframeState.servosChanged[ROLL] = 1;
		airframeState.servos[ROLL] = x;

	}

	x = map(joystickState.axis[YAW], -32767, 32767, 0, 1023);
	if(x != airframeState.servos[YAW]) {

		airframeState.servosChanged[YAW] = 1;
		airframeState.servos[YAW] = x;

	}

	x = map(joystickState.axis[PITCH], -32767, 32767, 0, 1023);
	if(x != airframeState.servos[PITCH]) {

		airframeState.servosChanged[PITCH] = 1;
		airframeState.servos[PITCH] = x;

	}

	time_t currentTime = time(NULL);

	if(difftime(currentTime, startTime) > THROTTLE_SAFETY_DEBUG) {  // Don't allow throttle to change for first 20 seconds after startup - DEBUG may change this?

		int currentThrottle = joystickState.axis[THROTTLE] * -1;

		if(currentThrottle < 0) {

			x = map(currentThrottle, -32767, -1 * configInfo.jsDiscardUnder, 0, 1023);

			if(x < airframeState.prevThrottle) {

				airframeState.servosChanged[THROTTLE] = 1;
				airframeState.servos[THROTTLE] = x;
				airframeState.prevThrottle = x;

			}

		} else if(currentThrottle > 0) {

			x = map(currentThrottle, configInfo.jsDiscardUnder, 32767, 0, 1023);

			if(x > airframeState.prevThrottle) {

				airframeState.servosChanged[THROTTLE] = 1;
				airframeState.servos[THROTTLE] = x;
				airframeState.prevThrottle = x;

			}

		}

/*		x = map(joystickState.axis[THROTTLE] * -1, -32767, 32767, 0, 1023); // Inverse throttle

		if(((joystickState.axis[THROTTLE] * -1) > configInfo.jsDiscardUnder && x > airframeState.prevThrottle) || ((joystickState.axis[THROTTLE] * -1) < (-1 * configInfo.jsDiscardUnder) && x < airframeState.prevThrottle)) {

			if(x != airframeState.servos[THROTTLE]) {
	
				airframeState.servosChanged[THROTTLE] = 1;
				airframeState.servos[THROTTLE] = x;
				airframeState.prevThrottle = x;
	
			}
	
		}*/
	
	}

	/*if(joystickState.axis[ROLL] > 0) {

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
	airframeState.servos[SRV_CAM_TILT] = map(joystickState.axis[CAM_TILT], -32767, 32767, 0, 180);*/

	for(x = 0 ; x < BUTTON_COUNT ; x++) {

		if(joystickState.button[x] != airframeState.buttons[x]) {

			airframeState.buttonsChanged[x] = 1;
			airframeState.buttons[x] = joystickState.button[x];

		}

	}

}

/* readJoystick(joystickState, configInfo) - Read joystick state from joystickState->port, update joystickState */

void readJoystick(jsState *joystickState, configValues configInfo) {

	struct js_event js;
	int jsValue;

	while(read(joystickState->port, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

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

				if(joystickState->button[configInfo.contextButton] == 0) { // Check the context we're working with

					joystickState->axis[js.number] = jsValue;  // Regular axis, just store the current value
				
				} else { 
									
					if(js.number == CAM_PAN_SRC) { // Handle cam pan axis

						joystickState->axis[CAM_PAN] = jsValue; // Store in CAM_PAN instead of js.number since we're in a different context
					
					} else if(js.number == CAM_TILT_SRC) {
									
						joystickState->axis[CAM_TILT] = jsValue; // Store in CAM_TILT since we're in a different context

					} else {

						joystickState->axis[js.number] = jsValue;  // Regular axis, just store the current value

					}		
				
				}
				break;

			}

		}

}

/* initMessage() - Initialise message */

int initMessage(messageState *message) {

	message->readBytes = 0;
	message->length = -1; // Init message.length as header length size

	if((message->messageBuffer = calloc(MSG_BUFFER_SIZE, sizeof(char))) != NULL) {
	
		return 1; // calloc() worked
		
	} else {
	
		return 0; // calloc() failed
	
	}	
	
}

/* checkXBeeMessages() - Check for and handle any incoming messages */

int checkXBeeMessages(int msgPort, messageState *msg) {

	unsigned char testByte = 0x00;

	if(msg->length == -1) { // Message has -1 length, check if anything in messageBuffer can fill that in

		msg->length = getMessageLength(msg);

	}

	if(msg->readBytes < msg->length || msg->length == -1) {  // We either aren't done reading the message or we don't have MSG_BEGIN and/or MTYPE and/or PARAM to tell us the real length

		if(read(msgPort, &testByte, sizeof(char)) == sizeof(char)) {  // We read a byte, so process it

			if(msg->readBytes == 0) { // Haven't got MSG_BEGIN yet, look for it

				if(testByte == MSG_BEGIN) { // Beginning of a messge

					msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
					msg->readBytes++;			       // Increment readBytes

					#if DEBUG_LEVEL == 3
					printf("MSG: %2x ", testByte);
					//printf("BYTE[%3d/%3d - HS:%d]: %2x\n", msg->readBytes, msg->length, signalInfo.handShook, testByte); // Print out each received byte	
					#endif		

				}
				#if DEBUG_LEVEL == 3
				else {

					printf("%2x\n", testByte);

				}
				#endif

			} else {

				msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
				msg->readBytes++;			       // Increment readBytes
				#if DEBUG_LEVEL == 3
				printf("%2x ", testByte);
				//printf("BYTE[%3d/%3d - HS:%d]: %2x\n", msg->readBytes, msg->length, signalInfo.handShook, testByte); // Print out each received byte	
				#endif	

			}

			return 1;

		} else {

 			return 0;

		}	

	} else { // Message is finished, process it

		#if DEBUG_LEVEL == 3
		printf("\n");
		#endif	

		if(msg->length > 0) { // Message is finished, process it

			if(testChecksum(msg->messageBuffer, msg->length)) { // Checksum passed, process message..  

				processMessage(msg);
				if(msg->messageBuffer[1] != MTYPE_PING) {
										
					signalInfo.lastMessageTime = time(NULL); // Set last message time, except for from a ping

				}

			} 

		} 

		int x;

		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

			msg->messageBuffer[x] = '\0';

		}

		msg->readBytes = 0;   // Zero out readBytes
		msg->length = -1;     // Set message length to -1

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
			msg->messageBuffer[1] = MTYPE_PPZ;
			msg->messageBuffer[2] = msg->length;
			msg->messageBuffer[msg->length - 1] = generateChecksum(msg->messageBuffer, msg->length - 1);
			writePortMsg(ports.xbeePort, "XBee", msg->messageBuffer, msg->length);

			msg->readBytes = PPZ_MSG_HEADER_SIZE;  // Leave room for header to be added
                        msg->length = PPZ_MSG_HEADER_SIZE;
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

/* getMessageLength(msg) */

int getMessageLength(messageState *msg) {

	if(msg->readBytes == 2) { // Do zero-parameter types first, if we can't find one, see if we have enough characters for one of the parametered types

		if(msg->messageBuffer[1] == MTYPE_HEARTBEAT) { // Do 3-char long messages first

			return MTYPE_HEARTBEAT_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_PING || msg->messageBuffer[1] == MTYPE_PING_REPLY) { // 4 char messages

			return MTYPE_PING_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_SINGLE_SERVO) {

			return MTYPE_SINGLE_SERVO_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_BUTTON_UPDATE) {

			return MTYPE_BUTTON_UPDATE_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_ALL_SERVOS) {
		
			return MTYPE_ALL_SERVOS_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_FULL_UPDATE) {
	
			return MTYPE_FULL_UPDATE_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_STATUS) {
	
			return MTYPE_STATUS_SZ;

		}  else {

			return -1; // Probably a parametered type or a bad message

		}

	} else if(msg->readBytes > 2) { // Didn't find any non-parameter message types, let's see if we have a parametered one

		if(msg->messageBuffer[1] == MTYPE_PPZ || msg->messageBuffer[1] == MTYPE_DEBUG) {

			int msgLength = (int)msg->messageBuffer[2];
			if(msgLength < MSG_BUFFER_SIZE) {
	
				return msgLength;  // PPZ & Debug messages have length as param

			} else {

				return -2; // Bogus message

			}

		} else if(msg->messageBuffer[1] == MTYPE_VAR_SERVOS) {
		
			int servoCount = (int)msg->messageBuffer[2];

			if(servoCount < SERVO_COUNT) {  // If we get close to SERVO_COUNT the sent message would be a ALL_SERVOS or FULL_UPDATE

				return 4 + servoCount * 2;  // Convert servo count to # of bytes (2 bytes per servo + begin + type + param + check)

			} else {

				return -2; // Bogus message
			}

		} else {

			return -2; // No valid message types to provide length found

		}

	} else {

		return -1; // Haven't read enough bytes yet

	}

}

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(messageState *msg) {

	unsigned char msgType = msg->messageBuffer[1];
	int x;

	if(msgType == MTYPE_PING) { // We got a ping, send an ack

		sendAck(msg);		

	} else if(msgType == MTYPE_PING_REPLY) {  // Handle the message, since it got past checksum it has to be legit

		if(msg->messageBuffer[2] == signalInfo.pingData) { //  See if the payload matches the ping packet we sent out
		
			signalInfo.handShook = 1;
			#if DEBUG_LEVEL == 3
			printf("Got ping reply w/ valid data!\n");
			#endif

		} else {

			#if DEBUG_LEVEL == 3
			printf("Got ping reply w/ invalid data!");
			#endif

		}

	} else if(msgType == MTYPE_PPZ) { // Handle PPZ message
	
		msg->messageBuffer[msg->length - 1] = '\0'; // Replace checksum with null
		for(x = 0 ; x < msg->length - PPZ_MSG_HEADER_SIZE ; x++) {
		
			msg->messageBuffer[x] = msg->messageBuffer[x + PPZ_MSG_HEADER_SIZE];  // Shift everything to the left MSG_HEADER_SIZE bytes
		
		}
		
		writePortMsg(ports.ppzPty.master, "PPZ", msg->messageBuffer, msg->length - PPZ_MSG_HEADER_SIZE); // Write out the message, minus the header size
	
	} else if(msgType == MTYPE_DEBUG) { // This is a debug message, print it
	
		#if DEBUG_LEVEL == 4
		time_t currentTime = time(NULL);
		time_t diff = currentTime - startTime;
		printf("DEBUG MSG from UAV [%lds since last]: ", diff);
		startTime = currentTime;
		#else
		printf("DEBUG MSG from UAV: ");
		#endif

		for(x = 3 ; x < msg->length - 1 ; x++) {
		
		
			printf("%c", msg->messageBuffer[x]);
		
		}
		printf("\n");
	} else if(msgType == MTYPE_STATUS) {

		// get remote RSSI & battery voltage from the message

		signalInfo.remoteRSSI = msg->messageBuffer[2];
		airframeState.batteryVoltage = msg->messageBuffer[3];

	}

}

/* writePortMsg(outputPort, portName, message, messageSize) - Write message to outputPort, deliver error if message fails to write */

void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize) {

	int msgWrote = 0;
	#if DEBUG_LEVEL == 6
	if(message[1] != MTYPE_HEARTBEAT) {

		int x;
		printf("WRITING[%02d]: ", messageSize);
		for(x = 0 ; x < messageSize ; x++) {
		
			printf("%2x ", message[x]);
	
		}
		printf("\n");
	}
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
		#if DEBUG_LEVEL == 5
		printf("++ Good checksum (length: %d): ", length);
		for(x = 0 ; x < length ; x++) {
				
			printf("%2x ", (unsigned int)message[x]); // Print the whole message in hex			
		
		}
		printf("CHK: "); // Print the checksum marker
		printf("%2x\n", checksum); // Print the checksum
		#endif	
		return 1;  // Checksum passed!

	} else {

		#if DEBUG_LEVEL == 5
		printf("-- Bad checksum (length: %d): ", length);
		for(x = 0 ; x < length ; x++) {
				
			printf("%2x ", (unsigned int)message[x]); // Print the whole message in hex			
		
		}
		printf("CHK: "); // Print the checksum marker
		printf("%2x\n", checksum); // Print the checksum
		#endif	
		return 0;

	}

}

/* sendCtrlUpdate() - Send latest control state to XBee port */

void sendCtrlUpdate(int signum) {

	signalInfo.ctrlCounter++;

	if(signalInfo.handShook) { // We're synced up, send a control update or heartbeat

		// first we figure out what, if anything, changed
		int servosChangedCount = 0;
		int sendButtons = 0;
		int servoUpdateIds[SERVO_COUNT];
		int servoUpdates[SERVO_COUNT];
		int x;

		if(signalInfo.ctrlCounter % 50 == 0) { // Send a full control update

			servosChangedCount = -1; // Set servosChangedCount to an impossible value to signal full update
			signalInfo.ctrlCounter = 0;

		} else { // Figure out the best kind of update to send

			// Gather counts for each type of control that changed

			for(x = 0 ; x < SERVO_COUNT ; x++) {

				if(airframeState.servosChanged[x]) {

					servoUpdates[servosChangedCount] = airframeState.servos[x];
					servoUpdateIds[servosChangedCount] = x;
					servosChangedCount++;
					airframeState.servosChanged[x] = 0;

				}

			}
	
			x = 0;

			while(x < BUTTON_COUNT) {

				if(airframeState.buttonsChanged[x]) {

					sendButtons = 1;
					airframeState.buttonsChanged[x] = 0;
					x = BUTTON_COUNT;
	
				}
				x++;

			}

		}

		int msgSize = 0;
		int msgType = 0;
		unsigned char *ctrlMsg;
		unsigned char msgData[MAX_CTRL_MSG_SZ];
		// Now figure out the best message type for our counts

		if(servosChangedCount == 1) { // Single servo update
	
			msgSize = MTYPE_SINGLE_SERVO_SZ;
			msgType = MTYPE_SINGLE_SERVO;
			#if DEBUG_LEVEL == 7
			printf("Servo[%d] updated to pos: %d\n", servoUpdateIds[0], servoUpdates[0]);
			#endif
			msgData[0] = (servoUpdateIds[0] << 2) & 252; // Shift the servo ID (6 bits) left 2 spaces, bitwise and with binary 1111 1100 to drop last 2 bits if they were filled in
			msgData[0] = msgData[0] | ((servoUpdates[0] >> 8) & 3); // Bitwise and with binary 0000 0011
			msgData[1] = servoUpdates[0] & 255; // Bitwise and with 255, binary 1111 1111, to get the real good stuffs & discard anything else
			#if DEBUG_LEVEL == 7
			printf("msgData[%d]: %2x\n", 0, msgData[0]); // Print hex
			printf("msgData[%d]: %2x\n", 1, msgData[1]); // Print hex
			#endif

		} else if(servosChangedCount > 1 && servosChangedCount < 8) { // Variable # of servos

			msgSize = 4 + (2 * servosChangedCount); // msgSize = 3 for header, 2 bytes each servo (10-bit pos + 6-bit servo number)
			msgType = MTYPE_VAR_SERVOS;
			msgData[0] = servosChangedCount;   // Store the # of servos as the first value

			#if DEBUG_LEVEL == 7
			printf("Updating %d servos\n", servosChangedCount);
			#endif

			for(x = 0 ; x < servosChangedCount ; x++) {

				msgData[(x * 2) + 1] = (servoUpdateIds[x] << 2) & 252; // Shift the servo ID (6 bits) left 2 spaces, bitwise and with binary 1111 1100 to drop last 2 bits if they were filled in
				msgData[(x * 2) + 1] = msgData[(x * 2) + 1] | ((servoUpdates[x] >> 8) & 3); // Bitwise and with binary 0000 0011
				msgData[(x * 2) + 2] = servoUpdates[x] & 255; // Bitwise and with 255, binary 1111 1111, to get the real good stuffs & discard anything else
				#if DEBUG_LEVEL == 7
				printf("Servo[%d] updated to pos: %d\n", servoUpdateIds[x], servoUpdates[x]);
				printf("msgData[%d]: %2x\n", x + 1, msgData[(x * 2) + 1]); // Print hex
				printf("msgData[%d]: %2x\n", x + 2, msgData[(x * 2) + 2]); // Print hex
				#endif

			}

		} else if(servosChangedCount > 7) { // All servos update

			msgSize = MTYPE_ALL_SERVOS_SZ;
			msgType = MTYPE_ALL_SERVOS;

			#if DEBUG_LEVEL == 7
			printf("Updating all servos\n");
			#endif

			for(x = 0 ; x < SERVO_COUNT ; x++) {
			
				msgData[(x * 2)] = (servoUpdateIds[x] << 2) & 252; // Shift the servo ID (6 bits) left 2 spaces, bitwise and with binary 1111 1100 to drop last 2 bits if they were filled in
				msgData[(x * 2)] = msgData[(x * 2)] | ((servoUpdates[x] >> 8) & 3); // Bitwise and with binary 0000 0011
				msgData[(x * 2) + 1] = servoUpdates[x] & 255; // Bitwise and with 255, binary 1111 1111, to get the real good stuffs & discard anything else
				#if DEBUG_LEVEL == 7
				printf("Servo[%d] updated to pos: %d\n", servoUpdateIds[x], servoUpdates[x]);
				printf("msgData[%d]: %2x\n", x * 2, msgData[(x * 2)]); // Print hex
				printf("msgData[%d]: %2x\n", (x * 2) + 1, msgData[(x * 2) + 1]); // Print hex
				#endif

			}
			

		} else if(servosChangedCount == -1) {

			sendButtons = 0; // Don't send buttons separately

			msgSize = MTYPE_FULL_UPDATE_SZ;
			msgType = MTYPE_FULL_UPDATE;

			#if DEBUG_LEVEL == 7
			printf("Sending full update\n");
			#endif

			for(x = 0 ; x < SERVO_COUNT ; x++) {
			
				msgData[(x * 2)] = (x << 2) & 252; // Shift the servo ID (6 bits) left 2 spaces, bitwise and with binary 1111 1100 to drop last 2 bits if they were filled in
				msgData[(x * 2)] = msgData[(x * 2)] | ((airframeState.servos[x] >> 8) & 3); // Bitwise and with binary 0000 0011
				msgData[(x * 2) + 1] = airframeState.servos[x] & 255; // Bitwise and with 255, binary 1111 1111, to get the real good stuffs & discard anything else
				#if DEBUG_LEVEL == 7
				printf("Servo[%d] updated to pos: %d\n", x, airframeState.servos[x]);
				printf("msgData[%d]: %2x\n", x * 2, msgData[(x * 2)]); // Print hex
				printf("msgData[%d]: %2x\n", (x * 2) + 1, msgData[(x * 2) + 1]); // Print hex
				#endif

			}

			for(x = 0 ; x < MTYPE_BUTTON_UPDATE_SZ - 3 ; x++) {
	
				msgData[x + 2 + (SERVO_COUNT * 2)] = (airframeState.buttons[(x * 4)] & 3) << 6; // Mask away anything but the last 2 bits and then bitshift to the left, this button is now now the 2 highest bits
				msgData[x + 2 + (SERVO_COUNT * 2)] = msgData[x + 2 + (SERVO_COUNT * 2)] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
				msgData[x + 2 + (SERVO_COUNT * 2)] = msgData[x + 2 + (SERVO_COUNT * 2)] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
				msgData[x + 2 + (SERVO_COUNT * 2)] = msgData[x + 2 + (SERVO_COUNT * 2)] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

			}

		} else if(servosChangedCount == 0 && !sendButtons && signalInfo.ctrlCounter % 25 == 0) { // Send only a heartbeat, 500ms interval
			
			msgSize = MTYPE_HEARTBEAT_SZ;
			msgType = MTYPE_HEARTBEAT;

		}

		if(msgSize > 0) {

			ctrlMsg = calloc(msgSize, sizeof(char)); // Allocate memory for our message
	
			ctrlMsg[0] = MSG_BEGIN; // First character of ctrlMsg is MSG_BEGIN
			ctrlMsg[1] = msgType;   // Message type determined above

			for(x = 0 ; x < (msgSize - 3) ; x++) { // msgSize minus three for the BEGIN, TYPE and CHECK bytes

				ctrlMsg[x + 2] = msgData[x];

			}

			ctrlMsg[msgSize - 1] = generateChecksum(ctrlMsg, msgSize - 1); // Store our checksum as our last byte

			#if DEBUG_LEVEL == 7
			if(msgType != MTYPE_HEARTBEAT && msgType != MTYPE_PING) {

				printf("Final Msg: ");
				for(x = 0 ; x < msgSize ; x++) {
	
					printf("%2x ", ctrlMsg[x]);			

				}
				printf("\n");
			}
			#endif 

			writePortMsg(ports.xbeePort, "XBee", ctrlMsg, msgSize); // Write out message to XBee
			free(ctrlMsg); // Deallocate memory for ctrlMsg	

		}

		if(sendButtons) { // Send buttons as a separate message, very rare that both this and the above code will execute in one go around

			ctrlMsg = calloc(MTYPE_BUTTON_UPDATE_SZ, sizeof(char));

			ctrlMsg[0] = MSG_BEGIN;
			ctrlMsg[1] = MTYPE_BUTTON_UPDATE;

			for(x = 0 ; x < MTYPE_BUTTON_UPDATE_SZ - 3 ; x++) {
	
				ctrlMsg[x + 2] = (airframeState.buttons[(x * 4)] & 3) << 6; // Mask away anything but the last 2 bits and then bitshift to the left, this button is now now the 2 highest bits
				ctrlMsg[x + 2] = ctrlMsg[x + 2] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
				ctrlMsg[x + 2] = ctrlMsg[x + 2] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
				ctrlMsg[x + 2] = ctrlMsg[x + 2] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

			}

			ctrlMsg[MTYPE_BUTTON_UPDATE_SZ - 1] = generateChecksum(ctrlMsg, MTYPE_BUTTON_UPDATE_SZ - 1);
			writePortMsg(ports.xbeePort, "XBee", ctrlMsg, MTYPE_BUTTON_UPDATE_SZ); // Write out message to XBee
			free(ctrlMsg);

		}

		signalInfo.totalMsgs++;
	
	} else {  // We aren't synced up, send ping request

		if(signalInfo.ctrlCounter % 5 == 0) { // Send these out more sparingly than regular updates

			unsigned char pingMsg[MTYPE_PING_SZ];
			signalInfo.pingData = rand() % 255; // Ping contains 1 byte that must be sent back as a valid ack
			signalInfo.ctrlCounter = 0;

			pingMsg[0] = MSG_BEGIN;
			pingMsg[1] = MTYPE_PING;
			pingMsg[2] = signalInfo.pingData;
			pingMsg[3] = generateChecksum(pingMsg, MTYPE_PING_SZ - 1); // Store our checksum as the last byte
	
			#if DEBUG_LEVEL == 0
			printf(".");
			#endif
			fflush(stdout);
			writePortMsg(ports.xbeePort, "XBee", pingMsg, MTYPE_PING_SZ);  // Write the handshake to the XBee port

		}

	}

}

/* sendAck(message) - Send ack! */

void sendAck(messageState *msg) {

	unsigned char pingReply[4];

	pingReply[0] = MSG_BEGIN;
	pingReply[1] = MTYPE_PING_REPLY;
	pingReply[2] = msg->messageBuffer[2];
	pingReply[3] = generateChecksum(pingReply, MTYPE_PING_REPLY_SZ - 1); // Store our checksum as the last byte

	writePortMsg(ports.xbeePort, "XBee", pingReply, MTYPE_PING_REPLY_SZ);  // Write the handshake to the XBee port

}


/* checkSignal() - Check whether the signal is good, if not RUMBLE!! */

int checkSignal(configValues configInfo, jsState *joystickState) {

	struct input_event play; // input_event control to play and stop effects
	time_t currentTime = time(NULL); // get current time

	if(signalInfo.handShook) { 

		printOutput();

		if(joystickState->rumbleLevel == EFFECTS_COUNT - 1) {
		
			joystickState->rumbleLevel = 0;
		
		}

		double signalTimeDiff = difftime(currentTime, signalInfo.lastMessageTime);

		if(signalTimeDiff > configInfo.timeoutThreshold) {  // Looks like we've lost our signal (>1s since last ping ack)

			signalInfo.handShook = 0;  // Turn off handshake so we can resync
			signalInfo.lostSignalTime = time(NULL); // Store time we lost the signal
			#if DEBUG_LEVEL != 4 && DEBUG_LEVEL != 5
			printf("Lost signal!  Attempting to resync.\n");
			#endif
			#if DEBUG_LEVEL == 1
			time_t diff = currentTime - startTime;
			printf("Total msgs: %lu Running: %lds\n", signalInfo.totalMsgs, diff);
			#endif
			return 0;

		}
		
	} else {

		double signalTimeDiff = difftime(currentTime, signalInfo.lostSignalTime);
		double rumbleTimeDiff = difftime(currentTime, joystickState->lastRumbleTime);
		
		if(signalTimeDiff > 0.0 && rumbleTimeDiff > 0.5 && joystickState->rumbleLevel < (EFFECTS_COUNT - 1)) {  // Small rumble
	
			joystickState->lastRumbleTime = currentTime;
			
			play.type = EV_FF;
			play.code = joystickState->effects[joystickState->rumbleLevel].id;
			play.value = 1;
			if (write(joystickState->event, (const void*) &play, sizeof(play)) == -1) {
				perror("Play effect");
				exit(1);
			}
			joystickState->rumbleLevel++;
			return 0;
	
		} else if(signalTimeDiff > 0.0 && rumbleTimeDiff > 0.5 && joystickState->rumbleLevel == (EFFECTS_COUNT - 1)) {
	
			joystickState->lastRumbleTime = currentTime;	
			play.type = EV_FF;
			play.code = joystickState->effects[joystickState->rumbleLevel].id;
			play.value = 1;

			if (write(joystickState->event, (const void*) &play, sizeof(play)) == -1) {
				perror("Play effect");
				exit(1);
			}		
			return 0;
		
		}
	
	}
	
	return 1; // Should never get here

}

/* printState(joystickState) - Debug function, prints the current joystick and airframe states */

void printState(jsState joystickState) {

	int i;
	printf("\033[2J\033[0;0H");

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

void printOutput() {

	time_t currentTime = time(NULL);

	if(difftime(currentTime, startTime) > (THROTTLE_SAFETY_DEBUG / 2.0)) {

		int x;
		printf("\033[2J\033[0;0H");

		printf("Joystick State: \n");

		printf("Button State:   \n");

		printf("Servo State:    ");

		for(x = 0 ; x < SERVO_COUNT ; x++) {

			printf("[%02d]: %4d  ", x, airframeState.servos[x] + 1);

		}
	
		printf("\n");

		printf("RSSI: %3d   Battery: %3d\n", signalInfo.remoteRSSI, airframeState.batteryVoltage);

		time_t currentTime = time(NULL);
		double diff = difftime(currentTime, startTime);

		if(diff < THROTTLE_SAFETY_DEBUG) {

			printf("Throttle Safety Remaining: %3.0f\n", THROTTLE_SAFETY_DEBUG - diff);

		}


		fflush(stdout);

	}

}
