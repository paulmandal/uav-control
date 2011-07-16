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
#include <ncurses.h>  // Let's use some f-cking curses!

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
#define VERSION_MINOR 3       // Minor #
#define VERSION_MOD   0       // Mod #
#define VERSION_TAG   "DBG"   // Tag

#define MAX_CTRL_MSG_SZ         19    // Maximum data size for a control update msg

#define MSG_BUFFER_SIZE     256      // Message buffer size in bytes
#define PPZ_MSG_HEADER_SIZE   3      // PPZ msg header size in bytes

#define NAME_LENGTH 128       // Length of Joystick name
#define CAM_PAN 6	      // Camera Pan axis #
#define CAM_PAN_SRC 3         // Camera Pan axis source
#define CAM_TILT 7            // Camera Tilt axis #
#define CAM_TILT_SRC 2        // Camera Tilt axis source
#define ROLL 3                // Roll axis #
#define PITCH 2               // Pitch axis #
#define YAW 0                 // Yaw axis #
#define THROTTLE 1            // Throttle axis #

#define SERVO_COUNT   8       // Total servos on airframe
#define BUTTON_COUNT 12       // Buttons on joystick

#define PPZ_MSG_SIZE_CTRL 1024
#define DISPLAY_BUFFER_SZ 4096

#define SRV_THROTTLE 0
#define SRV_LEFTWING 1
#define SRV_RIGHTWING 2
#define SRV_L_ELEVRON 3
#define SRV_R_ELEVRON 4
#define SRV_CAM_PAN 5
#define SRV_CAM_TILT 6
#define SRV_LANDING 7

#define CONFIG_FILE           "js_ctrl.rc" // Config file name
#define ENCODER_CONFIG_FILE   "encoder.rc" // Config file for PPM encoder
#define CONFIG_FILE_MIN_COUNT            8 // # of variables stored in config file 

#define EFFECTS_COUNT 16

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

	unsigned int *servos;
	unsigned int *buttons;
	int prevThrottle;
	int *servosChanged;
	int *buttonsChanged;
	int mainBatteryVoltage;
	int commBatteryVoltage;
	int videoBatteryVoltage;

} afState;

typedef struct _relayState { // Store the translated (servo + buttons) states, globally accessible

	int commBatteryVoltage;
	int videoBatteryVoltage;

} relayState;

typedef struct _encoderConfigValues {

	unsigned char debugPin;        // Pin for debug LED
	unsigned char statusLEDPin;    // Pin for status LED
	unsigned char navlightPin;     // Pin for navlight LED/LEDs
	unsigned char rssiPin;         // RSSI input pin
	unsigned char mainBatteryPin;  // Main battery input pin
	unsigned char commBatteryPin;  // Comm battery input pin (this board)
	unsigned char videoBatteryPin; // Video battery input pin
	int lostMessageThreshold;      // Time in ms without a message before assuming we've lost our signal
	int heartbeatInterval;         // Interval in ms to send our heartbeat
	int pingInterval;              // Interval in ms to send pings
	unsigned char servoCount;      // # of servos on this board
	unsigned char buttonCount;     // # of buttons on this board
	int ppmMinPulse;               // PPM min pulse length in 1/2 usec (default = 2000)
	int ppmMaxPulse;               // PPM max pulse length in 1/2 usec (default = 4000)
	int ppmHighPulse;              // PPM high pulse duration in 1/2 usec (default = 400)
	int ppmSyncPulse;              // PPM sync pulse length (bunch of math determines this)
	int statusIntervalSignalLost;  // Interval for status to flash when signal is lost
	int statusIntervalOK;          // Interval for status to flash when everything is OK
	int navlightInterval;          // Interval for navlights to flash
	int adcSampleRate;	       // Rate to sample ADC in, ms
	int voltageSamples;	       // How many voltage samples to average
	unsigned char *buttonPinMap;   // Mapping of buttons to output pins


} encoderConfigValues;

typedef struct _configValues {

	char *joystickEventFile; // joystickEvent filename (for RUMBLE!)
	char *joystickPortFile;  // joystickPort filename
	char *xbeePortFile;      // xbeePort filename
	int *buttonStateCount;   // Button state counts
	int jsDiscardUnder;	 // Joystick discard under threshold
	int ppmInterval;	 // Interval to send commands to XBee
	int pingInterval;        // Interval to send pings
	int heartbeatInterval;	 // Heartbeat interval
	int timeoutThreshold;    // How long without a message before timeout
	int contextButton;       // Button that affects joystick context
	int throttleSafety;	 // Throttle safety timeout in seconds

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
	int localdBm;		   // Local XBee RSSI in dBm
	int remotedBm;		   // UAV XBee RSSI in dBm
	int videodBm;		   // Video downlink RSSI in dBm
	int maxdBm;
	int mindBm;
	unsigned char pingData;    // Random character sent along with last ping
	int ctrlCounter;           // Counter for outbound control messages
	int sentConfig;		   // Have we sent the config yet?
	time_t lostSignalTime;     // Time signal was lost
	int initialConnection;     // Have we connected yet?

} signalState;

typedef struct _ncWindows {

	WINDOW *mainWindow;
	WINDOW *outputWindow;
	WINDOW *debugWindow;
	WINDOW *errorWindow;
	WINDOW *border1;
	WINDOW *border2;
	WINDOW *border3;

} ncWindows;

/* Numbers */

typedef enum _messageTypes {

	MTYPE_PING, 
	MTYPE_PING_REPLY, 
	MTYPE_HEARTBEAT, 
	MTYPE_FULL_UPDATE, 
	MTYPE_SINGLE_SERVO, 
	MTYPE_VAR_SERVOS, 
	MTYPE_ALL_SERVOS, 
	MTYPE_BUTTON_UPDATE, 
	MTYPE_PPZ, 
	MTYPE_DEBUG, 
	MTYPE_RESET, 
	MTYPE_STATUS, 
	MTYPE_GCS_STATUS,
	MTYPE_CONFIG,
	MTYPE_REQ_CFG,
	MTYPE_BEGIN

} messageTypes;

int messageSizes[] = {4, 4, 3, 22, 5, -1, 19, 6, -1, -1, -1, 11, 9, -1, 4, -1};

/* Let's do sum prototypes! */

int openPort(char *portName, char *use);
int openPty(ptyInfo *pty, char *use);
int openJoystick();
int readConfig();
void initGlobals();
void initTimer();
void initAirframe();
void initRumble();
void translateJStoAF();
void readJoystick();
int initMessage(messageState *message);
int checkXBeeMessages(int msgPort, messageState *msg);
int checkPPZMessages(int msgPort, messageState *msg);
void processMessage(messageState *msg);
int getMessageLength(messageState *msg);
void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize);
unsigned char generateChecksum(unsigned char *message, int length);
int testChecksum(unsigned char *message, int length);
void handleTimer(int signum);
int checkSignal();
void sendAck(messageState *msg);
void printState();
int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh);
char *fgetsNoNewline(char *s, int n, FILE *stream);
void printOutput();
void sendConfig();
void initRelay();
void rssiBars(int rssi, char *buffer);
void initNC();
void clearNC();

/* Global state storage variables */

// global variables to store states

afState airframeState;             // Current airframe state
jsState joystickState;             // Current joystick state
configValues configInfo;           // Configuration values
portState ports;	           // Port state holder
signalState signalInfo;            // Signal info
encoderConfigValues encoderConfig; // Encoder configuration
relayState relay;		   // Relay state info
ncWindows display;	           // ncurses windows

#if DEBUG_LEVEL > 0
int debugCommandsPerAck = 0;
#endif

time_t startTime;

/* Main function */

int main(int argc, char **argv)
{

	initNC();

	startTime = time(NULL);
	messageState xbeeMsg;	   // messageState for incoming XBee message
	messageState ppzMsg;	   // messageState for incoming PPZ message
	
	initGlobals();

	initMessage(&xbeeMsg);
	initMessage(&ppzMsg);
	
	ppzMsg.readBytes = PPZ_MSG_HEADER_SIZE; // Leave space for the addition of a header to the msg from GCS
	ppzMsg.length = PPZ_MSG_HEADER_SIZE;
	
	srand(time(NULL));  // Init random using current time
	if(readConfig() < 0) { // Read our config into our config vars

		perror("Error reading config"); // Error reading config file
		clearNC();
		return 1;

	}

	initAirframe();  // Init airframe state
	initRelay();     // Init relay state

	if((ports.xbeePort = openPort(configInfo.xbeePortFile, "XBee")) < 0) { // open the XBee port
		clearNC();
		return 1;
	}

	if(openPty(&ports.ppzPty, "PPZ") < 0) { // open the PPZ pty
		clearNC();
		return 1;	
		
	} 
	
	if(openJoystick() < 0) { // open the Joystick
		clearNC();
		return 1;
	}

	initRumble();   // Set up rumble effects
	initTimer(); 	// Set up timers
	wprintw(display.outputWindow, "\nPPZ pty file is: %s", ports.ppzPty.slaveDevice);
	wprintw(display.outputWindow, "Ready to read JS & relay for PPZ...\n\n");
	wrefresh(display.outputWindow);

	while(1) {

		if(!signalInfo.handShook) {

			wprintw(display.outputWindow, "Handshaking..");
			wrefresh(display.outputWindow);
			while(!signalInfo.handShook) {  // Handshaking loop

				checkXBeeMessages(ports.xbeePort, &xbeeMsg); // Check for pending msg bytes
				usleep(10); // Give 10usec for character to be removed from buffer by read()
				checkSignal(); // call checkSignal() to allow rumble

			}
	
		}
		
		int x;
		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {  // Try to read MSG_BUFFER_SIZE bytes per loop.. checkSignal() is the only thing we don't really need to do continually

			readJoystick();  // Check joystick for updates
		
			translateJStoAF();	// update Airframe model

			checkXBeeMessages(ports.xbeePort, &xbeeMsg); // Check for pending msg bytes
			//checkPPZMessages(ports.ppzPty.master, &ppzMsg); // Check for pending msg bytes
			usleep(10); // Pause for 10usec

		}

		checkSignal();  // Check if our signal is still good
	
	}

	free(xbeeMsg.messageBuffer); // Meh
	free(ppzMsg.messageBuffer); // Not that this matters..
	clearNC();
	return 0;

}

/* Function definitions */

/* initNC() - Initialise ncurses */

void initNC() {

	int rows, cols, x;

	display.mainWindow = initscr();
	cbreak();
	noecho();

	getmaxyx(display.mainWindow, rows, cols);

	int headerRows = 14;
	int outputRows = rows - 40;
	int debugRows = 5;
	int errorRows = 4;

	display.border1      = subwin(display.mainWindow,          1, cols, headerRows, 0);
	display.outputWindow = subwin(display.mainWindow, outputRows, cols, headerRows + 1, 0);
	display.border2      = subwin(display.mainWindow,          1, cols, headerRows + outputRows + 1, 0);
	display.debugWindow  = subwin(display.mainWindow,  debugRows, cols, headerRows + outputRows + 2, 0);
	display.border3      = subwin(display.mainWindow,          1, cols, headerRows + outputRows + debugRows + 2, 0);
	display.errorWindow  = subwin(display.mainWindow,  errorRows, cols, headerRows + outputRows + debugRows + 3, 0);

	idlok(display.outputWindow, TRUE);
	scrollok(display.outputWindow, TRUE);
	idlok(display.debugWindow, TRUE);
	scrollok(display.debugWindow, TRUE);
	idlok(display.errorWindow, TRUE);
	scrollok(display.errorWindow, TRUE);

	for(x = 0 ; x < (cols / 2) - 4; x++) {

		wprintw(display.border1, "-");

	}

	wprintw(display.border1, " Output ", rows);

	for(x = 0 ; x < (cols / 2) - 4; x++) {

		wprintw(display.border1, "-");

	}

	for(x = 0 ; x < (cols / 2) - 2; x++) {

		wprintw(display.border2, "-");

	}

	wprintw(display.border2, " Debug ");

	for(x = 0 ; x < (cols / 2) - 2; x++) {

		wprintw(display.border2, "-");

	}
	for(x = 0 ; x < (cols / 2) - 4; x++) {

		wprintw(display.border3, "-");

	}

	wprintw(display.border3, " Error ");

	for(x = 0 ; x < (cols / 2) - 2; x++) {

		wprintw(display.border3, "-");

	}
	
	wrefresh(display.mainWindow);
	wrefresh(display.border1);
	wrefresh(display.outputWindow);
	wrefresh(display.border2);
	wrefresh(display.debugWindow);
	wrefresh(display.border3);
	wrefresh(display.errorWindow);

}

void clearNC() {

	endwin();

}

/* initGlobals() - initalise all global vars */

void initGlobals() {

	signalInfo.handShook = 0;
	signalInfo.localRSSI = 0;
	signalInfo.remoteRSSI = 0;
	signalInfo.videoRSSI = 0;
	signalInfo.ctrlCounter = 0;
	signalInfo.sentConfig = 0;
	signalInfo.initialConnection = 0;
	signalInfo.lastMessageTime = time(NULL);
	signalInfo.lostSignalTime = time(NULL);

	signalInfo.maxdBm = -1000;
	signalInfo.mindBm = 0;
	
}

/* openPort(portName, use) - Open a UART portName for usage use */

int openPort(char *portName, char *use) {

	int fd;
	wprintw(display.outputWindow, "Opening serial port %s for %s..\n", portName, use);
	wrefresh(display.outputWindow);

	if ((fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {  // Try to open the port
		printf("Port attempted: %s for %s\n", portName, use);
		perror("Error opening port");  // If there's an error, blow up!
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

	wprintw(display.outputWindow, "Opening pty for %s..\n", use);
	wrefresh(display.outputWindow);
	if((pty->master = posix_openpt(O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {  // Create our pty with posix_openpt()
	
		perror("Error opening pty");
		return 0;
	
	}
	
	if((grantpt(pty->master) == -1) || (unlockpt(pty->master) == -1) || ((pty->slaveDevice = ptsname(pty->master)) == NULL)) { // Grant permissions and unlock our pty, then return the device name for display to the user
	
		perror("Error unlocking pty");
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

int openJoystick() {

	uint16_t btnmap[BTNMAP_SIZE];
	uint8_t axmap[AXMAP_SIZE];
	int btnmapok = 1;
	int version = 0x000800;
	char name[NAME_LENGTH] = "Unknown";
	int fd, efd, i;

	wprintw(display.outputWindow, "Opening joystick %s..\n", configInfo.joystickPortFile);
	wrefresh(display.outputWindow);
	if ((fd = open(configInfo.joystickPortFile, O_RDWR | O_NONBLOCK)) < 0) {  // Open joystick port in non-blocking mode
		perror("Error opening joystick port");  // Error opening port
		return -1;
	}

	ioctl(fd, JSIOCGVERSION, &version);        // Get the joystick driver version
	ioctl(fd, JSIOCGAXES, &joystickState.axes);              // Get the axes count
	ioctl(fd, JSIOCGBUTTONS, &joystickState.buttons);        // Get the button count
	ioctl(fd, JSIOCGNAME(NAME_LENGTH), name);  // Get the joystick name

	joystickState.axes = SERVO_COUNT;
	joystickState.axis = calloc(SERVO_COUNT, sizeof(int));        // Allocate memory for joystick axes
	joystickState.button = calloc(joystickState.buttons, sizeof(char));  // Allocate memory for joystick buttons

	getaxmap(fd, axmap);   // Get the axis map
	getbtnmap(fd, btnmap); // Get the button map

	/* Determine whether the button map is usable. */
	for (i = 0; btnmapok && i < joystickState.buttons; i++) {
		if (btnmap[i] < BTN_MISC || btnmap[i] > KEY_MAX) {
			btnmapok = 0;
			break;
		}
	}
	if (!btnmapok) {
		/* btnmap out of range for names. Don't print any. */
		//puts("js_ctrl is not fully compatible with your kernel. Unable to retrieve button map!");
		//printf("Joystick (%s) has %d axes ", name, joystickState.axes);
		//printf("and %d buttons.\n", joystickState.buttons);
	} else {
		//printf("Joystick (%s) initialised with %d axes and %d buttons.\n", name, joystickState.axes, joystickState.buttons);  // Button map is OK, print joystick info
	}

	wprintw(display.outputWindow, "Opening joystick event file %s..\n", configInfo.joystickEventFile);
	wrefresh(display.outputWindow);
	if ((efd = open(configInfo.joystickEventFile, O_RDWR)) < 0) {  // Open joystick port in non-blocking mode
		perror("Error opening joystick event");  // Error opening port
		return -1;
	}

	joystickState.rumbleLevel = 0;
	joystickState.lastRumbleTime = time(NULL);

	joystickState.port = fd;   // Store joystick Port file descriptor
	joystickState.event = efd; // Store joystick Event file descriptor
	return 0;

}

/* readConfig() - Read values from configuration file */

int readConfig() {

	FILE *fp;
	int buttonCount = 1, readCount = 0, lineBuffer = 1024;
	char line[lineBuffer];
	if ((fp = fopen(CONFIG_FILE, "r")) == NULL) {  // Open the config file read-only
		
		return -1;  // Return -1 if error opening

	} else {

		while (fgetsNoNewline(line, lineBuffer, fp) != NULL) {  // Read the entire file checking it line by line

			if(strcmp(line, "[XBee Port File]") == 0) {  // Line matches a config variable header, read the next line (value) and store it
		
				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {
					
					configInfo.xbeePortFile = calloc(strlen(line) + 1, sizeof(char)); // Allocate memory for our variable
					strcpy(configInfo.xbeePortFile, line);                            // Copy value into our var
					readCount++;                                                       // Increment our value count

				}

			} else if(strcmp(line, "[Joystick Port File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.joystickPortFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(configInfo.joystickPortFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Event File]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.joystickEventFile = calloc(strlen(line) + 1, sizeof(char));
					strcpy(configInfo.joystickEventFile, line);
					readCount++;

				}

			} else if(strcmp(line, "[Joystick Discard Threshold]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.jsDiscardUnder = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[PPM Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.ppmInterval = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Context Button]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.contextButton = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Timeout Threshold]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.timeoutThreshold = atoi(line); // Translate ASCII -> double
					readCount++;

				}

			} else if(strcmp(line, "[Throttle Safety]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.throttleSafety = atoi(line); // Translate ASCII -> double
					readCount++;

				}

			} else if(strcmp(line, "[Heartbeat Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.heartbeatInterval = atoi(line); // Translate ASCII -> double
					readCount++;

				}

			} else if(strcmp(line, "[Ping Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					configInfo.pingInterval = atoi(line); // Translate ASCII -> double
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

					configInfo.buttonStateCount = calloc(buttonCount, sizeof(int)); // Allocate memory for our button state counts

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
						configInfo.buttonStateCount[currButton] = atoi(buttonBuffer);
						currButton++;

					}
					
				}

			}

		}

	}

	fclose(fp);

	// Reading encoder config file

	if ((fp = fopen(ENCODER_CONFIG_FILE, "r")) == NULL) {  // Open the config file read-only
		
		return -1;  // Return -1 if error opening

	} else {

		while (fgetsNoNewline(line, lineBuffer, fp) != NULL) {  // Read the entire file checking it line by line

			if(strcmp(line, "[Debug Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.debugPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Status LED Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.statusLEDPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Navlight Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.navlightPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[RSSI Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.rssiPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Main Battery Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.mainBatteryPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Comm Battery Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.commBatteryPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Video Battery Pin]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.videoBatteryPin = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Lost Message Threshold]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.lostMessageThreshold = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Heartbeat Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.heartbeatInterval = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Ping Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.pingInterval = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[PPM Min Pulse]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.ppmMinPulse = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[ADC Sample Rate]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.adcSampleRate = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Voltage Samples]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.voltageSamples = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Servo Count]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.servoCount = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[Button Count]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.buttonCount = atoi(line); // Translate ASCII -> int
					readCount++;

				}

			} else if(strcmp(line, "[PPM Max Pulse]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.ppmMaxPulse = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[PPM High Pulse]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.ppmHighPulse = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[PPM Sync Pulse]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.ppmSyncPulse = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Status Interval Signal Lost]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.statusIntervalSignalLost = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Status Interval OK]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.statusIntervalOK = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Navlight Interval]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					encoderConfig.navlightInterval = atoi(line); // Translate ASCII -> int
					readCount++;
	
				}

			} else if(strcmp(line, "[Button Pin Map]") == 0) {

				if(fgetsNoNewline(line, lineBuffer, fp) != NULL) {

					int bufferPos, currButton, strPos;
					char buttonBuffer[4]; // More than 3 digits of a button mapping
					int lineLength = strlen(line);
					for(strPos = 0 ; strPos < lineLength ; strPos++) { // Iterate through line[] character by character to get the button count

						if(line[strPos] == ',') { // Each comma separates a button mapping so totalling them should get us our button count

							buttonCount++;

						}
					}

					encoderConfig.buttonPinMap = calloc(encoderConfig.buttonCount, sizeof(int)); // Allocate memory for our button mapping

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
						encoderConfig.buttonPinMap[currButton] = atoi(buttonBuffer);
						currButton++;

					}
	
				}

			}

		}

	}

	if(readCount >= CONFIG_FILE_MIN_COUNT) {

		return 1;  // Read enough config variables, success

	} else {

		return 0;  // Didn't read enough of config values

	}	

}

/* initTimer() - set up timer */

void initTimer() {
 
	struct sigaction saCtrl;
	struct itimerval timerCtrl;

	wprintw(display.outputWindow, "Setting up pulse timer..\n");
	wrefresh(display.outputWindow);

	memset(&saCtrl, 0, sizeof (saCtrl));                       // Make signal object
	saCtrl.sa_handler = &handleTimer;                   // Set signal function handler in 'saCtrl'
	sigaction(SIGALRM, &saCtrl, NULL);                     // Set SIGALRM signal handler

	timerCtrl.it_value.tv_sec = 0;
	timerCtrl.it_value.tv_usec = 1000;    // Set timer interval to 1000usec = 1ms
	timerCtrl.it_interval.tv_sec = 0;
	timerCtrl.it_interval.tv_usec = 1000; // Set timer reset to 1ms

	wprintw(display.outputWindow, "Starting pulse timer..\n");
	wrefresh(display.outputWindow);

	setitimer(ITIMER_REAL, &timerCtrl, NULL);               // Start the timer


}

/* initAirframe() - Initalise relay state */

void initRelay() {

	relay.commBatteryVoltage = 0;
	relay.videoBatteryVoltage = 0;

}

/* initAirframe() - Initalise airframe state */

void initAirframe() {

	airframeState.servos = calloc(encoderConfig.servoCount, sizeof(int));
	airframeState.buttons = calloc(encoderConfig.buttonCount, sizeof(int));
	airframeState.servosChanged = calloc(encoderConfig.servoCount, sizeof(int));
	airframeState.buttonsChanged = calloc(encoderConfig.buttonCount, sizeof(int));
	int i;

	for(i = 0 ; i < encoderConfig.servoCount ; i++) {

		airframeState.servos[i] = 511;  // Set all servo pos to 511

	}

	airframeState.servos[THROTTLE] = 0;
	airframeState.mainBatteryVoltage = 0;
	airframeState.commBatteryVoltage = 0;
	airframeState.videoBatteryVoltage = 0;
	airframeState.prevThrottle = 0;

}

/* initRumble() - Initalise rumble effects */

void initRumble() {

	int x;
	int strongMagnitude;
	int weakMagnitude;
	
	for(x = 0 ; x < EFFECTS_COUNT ; x++) {
		
		strongMagnitude = x < 3 ? 0 : (0xffff / 12) * (x - 3);
		weakMagnitude = (0xffff / 8) * (x + 1);
		joystickState.effects[x].type = FF_RUMBLE;
		joystickState.effects[x].id = -1;
		joystickState.effects[x].u.rumble.strong_magnitude = strongMagnitude;
		joystickState.effects[x].u.rumble.weak_magnitude = weakMagnitude > 0xffff ? 0xffff : weakMagnitude;
		joystickState.effects[x].replay.length = 1000;
		joystickState.effects[x].replay.delay = 0;
		
		if (ioctl(joystickState.event, EVIOCSFF, &joystickState.effects[x]) == -1) {
			perror("Upload effects[x]");
		}
	
	}

}

/* translateJStoAF() - Translate current joystick settings to airframe settings (e.g. axis -> servos) */

void translateJStoAF() {

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

	x = map(joystickState.axis[CAM_PAN], -32767, 32767, 0, 1023);
	if(x != airframeState.servos[CAM_PAN]) {

		airframeState.servosChanged[CAM_PAN] = 1;
		airframeState.servos[CAM_PAN] = x;

	}

	x = map(joystickState.axis[CAM_TILT], -32767, 32767, 0, 1023);
	if(x != airframeState.servos[CAM_TILT]) {

		airframeState.servosChanged[CAM_TILT] = 1;
		airframeState.servos[CAM_TILT] = x;

	}

	time_t currentTime = time(NULL);

	if(difftime(currentTime, startTime) > (double)configInfo.throttleSafety) {  // Don't allow throttle to change for first x seconds

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

/* readJoystick(joystickState, configInfo) - Read joystick state from joystickState.port, update joystickState */

void readJoystick() {

	struct js_event js;
	int jsValue;

	while(read(joystickState.port, &js, sizeof(struct js_event)) == sizeof(struct js_event)) {

			switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:

				if(configInfo.buttonStateCount[js.number] > 0) {  // Check if the button is a toggle or not

					if(js.value == 1) {  // If it is a multi-state and this event is a button press, cycle through states

						if(joystickState.button[js.number] < configInfo.buttonStateCount[js.number]) {

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

				if(abs(js.value) > configInfo.jsDiscardUnder) {  // If the value is greater than our discard value set it
			
					jsValue = js.value;
					
				} else {

					jsValue = 0;  // Disregard values less than configInfo.jsDiscardUnder to avoid excessively sensitive sticks

				}

				if(joystickState.button[configInfo.contextButton] == 0) { // Check the context we're working with

					joystickState.axis[js.number] = jsValue;  // Regular axis, just store the current value
				
				} else { 
									
					if(js.number == CAM_PAN_SRC) { // Handle cam pan axis

						joystickState.axis[CAM_PAN] = jsValue; // Store in CAM_PAN instead of js.number since we're in a different context
					
					} else if(js.number == CAM_TILT_SRC) {
									
						joystickState.axis[CAM_TILT] = jsValue; // Store in CAM_TILT since we're in a different context

					} else {

						joystickState.axis[js.number] = jsValue;  // Regular axis, just store the current value

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

	if(msg->readBytes < msg->length || msg->length == -1) {  // We either aren't done reading the message or we don't have MTYPE_BEGIN and/or MTYPE and/or PARAM to tell us the real length

		if(read(msgPort, &testByte, sizeof(char)) == sizeof(char)) {  // We read a byte, so process it

			if(msg->readBytes == 0) { // Haven't got MTYPE_BEGIN yet, look for it

				if(testByte == MTYPE_BEGIN) { // Beginning of a messge

					msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
					msg->readBytes++;			       // Increment readBytes

					#if DEBUG_LEVEL == 3
					wprintw(display.debugWindow, "MSG: %2x ", testByte);
					wrefresh(display.debugWindow);
					//printf("BYTE[%3d/%3d - HS:%d]: %2x\n", msg->readBytes, msg->length, signalInfo.handShook, testByte); // Print out each received byte	
					#endif		

				}
				#if DEBUG_LEVEL == 3
				else {

					wprintw(display.debugWindow, "%2x\n", testByte);
					wrefresh(display.debugWindow);
				}
				#endif

			} else {

				msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
				msg->readBytes++;			       // Increment readBytes
				#if DEBUG_LEVEL == 3
				wprintw(display.debugWindow, "%2x ", testByte);
				wrefresh(display.debugWindow);
				//printf("BYTE[%3d/%3d - HS:%d]: %2x\n", msg->readBytes, msg->length, signalInfo.handShook, testByte); // Print out each received byte	
				#endif	

			}

			return 1;

		} else {

 			return 0;

		}	

	} else { // Message is finished, process it

		#if DEBUG_LEVEL == 3
		wprintw(display.debugWindow, "\n");
		wrefresh(display.debugWindow);
		#endif	

		if(msg->length > 0) { // Message is finished, process it

			if(testChecksum(msg->messageBuffer, msg->length)) { // Checksum passed, process message..  

				processMessage(msg);
				if(msg->messageBuffer[1] != MTYPE_PING && msg->messageBuffer[1] != MTYPE_GCS_STATUS ) {
										
					signalInfo.lastMessageTime = time(NULL); // Set last message time, except for from a ping or GCS status update

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
		
			msg->messageBuffer[0] = MTYPE_BEGIN;
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
		
		if(msg->messageBuffer[1] < MTYPE_BEGIN) {

			int size = messageSizes[msg->messageBuffer[1]];

			return size; // We got the message size or its a parametered type

		} else {

			return -2; // Invalid type

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
		
			int numServos = (int)msg->messageBuffer[2];

			if(numServos < SERVO_COUNT) {  // If we get close to SERVO_COUNT the sent message would be a ALL_SERVOS or FULL_UPDATE

				return 4 + numServos * 2;  // Convert servo count to # of bytes (2 bytes per servo + begin + type + param + check)

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

		wprintw(display.outputWindow, "..connection complete!\n");
		wrefresh(display.outputWindow);

		sendAck(msg);		

	} else if(msgType == MTYPE_PING_REPLY) {  // Handle the message, since it got past checksum it has to be legit

		if(msg->messageBuffer[2] == signalInfo.pingData) { //  See if the payload matches the ping packet we sent out
		
			signalInfo.handShook = 1;
			signalInfo.initialConnection = 1;

		} else {

			wprintw(display.outputWindow, ".Got ping reply w/ invalid data..");
			wrefresh(display.outputWindow);

		}

	} else if(msgType == MTYPE_PPZ) { // Handle PPZ message
	
		msg->messageBuffer[msg->length - 1] = '\0'; // Replace checksum with null
		for(x = 0 ; x < msg->length - PPZ_MSG_HEADER_SIZE ; x++) {
		
			msg->messageBuffer[x] = msg->messageBuffer[x + PPZ_MSG_HEADER_SIZE];  // Shift everything to the left MSG_HEADER_SIZE bytes
		
		}
		
		writePortMsg(ports.ppzPty.master, "PPZ", msg->messageBuffer, msg->length - PPZ_MSG_HEADER_SIZE); // Write out the message, minus the header size
	
	} else if(msgType == MTYPE_DEBUG) { // This is a debug message, print it
	
		wprintw(display.outputWindow, "DEBUG MSG from UAV: ");

		for(x = 3 ; x < msg->length - 1 ; x++) {
		
		
			wprintw(display.outputWindow, "%c", msg->messageBuffer[x]);
		
		}
		wprintw(display.outputWindow, "\n");
		wrefresh(display.outputWindow);

	} else if(msgType == MTYPE_REQ_CFG) {

		sendConfig();

	} else if(msgType == MTYPE_STATUS) {

		// get remote RSSI & battery voltage from the message
		//snprintf(printBuffer, DISPLAY_BUFFER_SZ, "%sGot UAV status msg..", debugBuffer);
		//strcpy(debugBuffer, printBuffer);
		signalInfo.remoteRSSI = msg->messageBuffer[2] << 8;
		signalInfo.remoteRSSI = signalInfo.remoteRSSI | msg->messageBuffer[3];
		airframeState.mainBatteryVoltage = msg->messageBuffer[4] << 8;
		airframeState.mainBatteryVoltage = airframeState.mainBatteryVoltage | msg->messageBuffer[5];
		airframeState.commBatteryVoltage = msg->messageBuffer[6] << 8;
		airframeState.commBatteryVoltage = airframeState.commBatteryVoltage | msg->messageBuffer[7];
		airframeState.videoBatteryVoltage = msg->messageBuffer[8] << 8;
		airframeState.videoBatteryVoltage = airframeState.videoBatteryVoltage | msg->messageBuffer[9];
		wprintw(display.debugWindow, "\nUAV RSSI %4d MBATT %4d CBATT %4d VBATT %4d", signalInfo.remoteRSSI, airframeState.mainBatteryVoltage, airframeState.commBatteryVoltage, airframeState.videoBatteryVoltage);
		wrefresh(display.debugWindow);

		signalInfo.remotedBm = ((signalInfo.remoteRSSI + 5928) / 41) - 256;

	} else if(msgType == MTYPE_GCS_STATUS) {

		// get remote RSSI & battery voltage from the message
		//snprintf(printBuffer, DISPLAY_BUFFER_SZ, "%sGot GCS status msg..", debugBuffer);
		//strcpy(debugBuffer, printBuffer);
		signalInfo.localRSSI = msg->messageBuffer[2] << 8;
		signalInfo.localRSSI = signalInfo.localRSSI | msg->messageBuffer[3];
		relay.commBatteryVoltage = msg->messageBuffer[4] << 8;
		relay.commBatteryVoltage = relay.commBatteryVoltage | msg->messageBuffer[5];
		relay.videoBatteryVoltage = msg->messageBuffer[6] << 8;
		relay.videoBatteryVoltage = relay.videoBatteryVoltage | msg->messageBuffer[7];
		wprintw(display.debugWindow, "\nGCS RSSI %4d CBATT %4d VBATT %4d", signalInfo.localRSSI, relay.commBatteryVoltage, relay.videoBatteryVoltage);
		wrefresh(display.debugWindow);

		signalInfo.localdBm = ((signalInfo.localRSSI + 5928) / 41) - 256;
		if(signalInfo.localdBm > signalInfo.maxdBm) { signalInfo.maxdBm = signalInfo.localdBm; }
		if(signalInfo.localdBm < signalInfo.mindBm) { signalInfo.mindBm = signalInfo.localdBm; }	

	}

}

/* writePortMsg(outputPort, portName, message, messageSize) - Write message to outputPort, deliver error if message fails to write */

void writePortMsg(int outputPort, char *portName, unsigned char *message, int messageSize) {

	int msgWrote = 0;
	#if DEBUG_LEVEL == 6
	if(message[1] != MTYPE_HEARTBEAT) {

		int x;
		wprintw(display.debugWindow, "WRITING[%02d]: ", messageSize);

		for(x = 0 ; x < messageSize ; x++) {
		
			wprintw(display.debugWindow, "%2x ", message[x]);
	
		}
		wprintw(display.debugWindow, "\n");
		wrefresh(display.debugWindow);

	}
	#endif
	msgWrote = write(outputPort, message, messageSize);  // write() and store written byte count in msgWrote
	if(msgWrote != messageSize) { // If written byte count is not expected value
				
		wprintw(display.errorWindow, "error writing to %s, wrote: %d/%d bytes.\n", portName, msgWrote, messageSize);  // Output error and info on what happened
		wrefresh(display.errorWindow);

	}

}

/* generateChecksum(message, length) - Generate a checksum for message */

unsigned char generateChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 21
	wprintw(display.debugWindow, "GENCHK[%2d]: ", length);
	for(x = 0 ; x < length ; x++) {

		wprintw(display.debugWindow, "%2x ", (unsigned int)message[x]); // Print the whole message in hex

	}
	wprintw(display.debugWindow, "CHK: "); // Print the checksum marker
	#endif	

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x]; // Generate checksum

	}

	#if DEBUG_LEVEL == 21
	wprintw(display.debugWindow, "%2x\n\n", checksum); // Print the checksum
	wrefresh(display.debugWindow);
	#endif	

	return checksum;

}

/* testChecksum(message, length) - Test if the last byte checksum is good */

int testChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 3
	wprintw(display.debugWindow, "CHKMSG[%2d]: ", length);

	for(x = 0 ; x < length ; x++) {

		wprintw(display.debugWindow, "%2x ", (unsigned int)message[x]); // Print the whole message in hex

	}
	wprintw(display.debugWindow, "CHK: "); // Print the checksum marker
	#endif	

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x];  // Test the message against its checksum (last byte)

	}

	#if DEBUG_LEVEL == 3
	wprintw(display.debugWindow, "%2x\n\n", checksum); // Print the checksum
	wrefresh(display.debugWindow);
	#endif	

	if(checksum == 0x00) {

		#if DEBUG_LEVEL == 5
		wprintw(display.debugWindow, "++ Good checksum (length: %d): ", length);

		for(x = 0 ; x < length ; x++) {
				
			wprintw(display.debugWindow, "%2x ", (unsigned int)message[x]); // Print the whole message in hex			
		
		}
		wprintw(display.debugWindow, "CHK: %2x\n", checksum); // Print the checksum
		wrefresh(display.debugWindow);
		#endif	
		return 1;  // Checksum passed!

	} else {

		#if DEBUG_LEVEL == 5
		wprintw(display.debugWindow, "-- Bad checksum (length: %d): ", length);

		for(x = 0 ; x < length ; x++) {
				
			wprintw(display.debugWindow, "%2x ", (unsigned int)message[x]); // Print the whole message in hex			
		
		}
		wprintw(display.debugWindow, "CHK: %2x\n", checksum); // Print the checksum
		wrefresh(display.debugWindow);
		#endif	
		return 0;

	}

}

/* handleTimer() - Exec every 1ms, handle anything that needs tight timing */

void handleTimer(int signum) {

	signalInfo.ctrlCounter++; // Increment the counter

	if(signalInfo.ctrlCounter % (configInfo.ppmInterval * 2) == 0) {

		printOutput(); // Print output every other ppmInterval, shouldn't be too fast

	}

	if(signalInfo.handShook) { // We're synced up, send a control update or heartbeat

		if(signalInfo.sentConfig) { // We've sent the config, so send an update or heartbeat	

		// first we figure out what, if anything, changed
		int servosChangedCount = 0;
		int sendButtons = 0;
		int servoUpdateIds[SERVO_COUNT];
		int servoUpdates[SERVO_COUNT];
		int x;

		if(signalInfo.ctrlCounter % (configInfo.ppmInterval * 50) == 0) { // Send a full control update

			servosChangedCount = -1; // Set servosChangedCount to an impossible value to signal full update
			signalInfo.ctrlCounter = 0;

		} else if(signalInfo.ctrlCounter % configInfo.ppmInterval == 0) { // Figure out the best kind of update to send		

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
	
			msgSize = messageSizes[MTYPE_SINGLE_SERVO];
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

			msgSize = messageSizes[MTYPE_ALL_SERVOS];
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

			msgSize = messageSizes[MTYPE_FULL_UPDATE];
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

			for(x = 0 ; x < messageSizes[MTYPE_BUTTON_UPDATE] - 3 ; x++) {
	
				msgData[x + 2 + (SERVO_COUNT * 2)] = (airframeState.buttons[(x * 4)] & 3) << 6; // Mask away anything but the last 2 bits and then bitshift to the left, this button is now now the 2 highest bits
				msgData[x + 2 + (SERVO_COUNT * 2)] = msgData[x + 2 + (SERVO_COUNT * 2)] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
				msgData[x + 2 + (SERVO_COUNT * 2)] = msgData[x + 2 + (SERVO_COUNT * 2)] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
				msgData[x + 2 + (SERVO_COUNT * 2)] = msgData[x + 2 + (SERVO_COUNT * 2)] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

			}

		} else if(servosChangedCount == 0 && !sendButtons && signalInfo.ctrlCounter % configInfo.heartbeatInterval == 0) { // Send only a heartbeat, 500ms interval
			
			msgSize = messageSizes[MTYPE_HEARTBEAT];
			msgType = MTYPE_HEARTBEAT;

		}

		if(msgSize > 0) {

			ctrlMsg = calloc(msgSize, sizeof(char)); // Allocate memory for our message
	
			ctrlMsg[0] = MTYPE_BEGIN; // First character of ctrlMsg is MTYPE_BEGIN
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

			ctrlMsg = calloc(messageSizes[MTYPE_BUTTON_UPDATE], sizeof(char));

			ctrlMsg[0] = MTYPE_BEGIN;
			ctrlMsg[1] = MTYPE_BUTTON_UPDATE;

			for(x = 0 ; x < messageSizes[MTYPE_BUTTON_UPDATE] - 3 ; x++) {
	
				ctrlMsg[x + 2] = (airframeState.buttons[(x * 4)] & 3) << 6; // Mask away anything but the last 2 bits and then bitshift to the left, this button is now now the 2 highest bits
				ctrlMsg[x + 2] = ctrlMsg[x + 2] | (airframeState.buttons[(x * 4) + 1] & 3) << 4;  // Mask away same, bitshift 4 to the left and bitwise OR to add this to our byte
				ctrlMsg[x + 2] = ctrlMsg[x + 2] | (airframeState.buttons[(x * 4) + 2] & 3) << 2;  // Same
				ctrlMsg[x + 2] = ctrlMsg[x + 2] | (airframeState.buttons[(x * 4) + 3] & 3);       // Same, no bitshift since we're already on the last two bits

			}

			ctrlMsg[messageSizes[MTYPE_BUTTON_UPDATE] - 1] = generateChecksum(ctrlMsg, messageSizes[MTYPE_BUTTON_UPDATE] - 1);
			writePortMsg(ports.xbeePort, "XBee", ctrlMsg, messageSizes[MTYPE_BUTTON_UPDATE]); // Write out message to XBee
			free(ctrlMsg);

		}

		}

	} else {  // We aren't synced up, send ping request

		if(signalInfo.ctrlCounter % configInfo.pingInterval == 0) { // Send these out more sparingly than regular updates

			unsigned char *pingMsg;
			pingMsg = calloc(messageSizes[MTYPE_PING], sizeof(char));
			signalInfo.pingData = rand() % 255; // Ping contains 1 byte that must be sent back as a valid ack
			signalInfo.ctrlCounter = 0;

			pingMsg[0] = MTYPE_BEGIN;
			pingMsg[1] = MTYPE_PING;
			pingMsg[2] = signalInfo.pingData;
			pingMsg[3] = generateChecksum(pingMsg, messageSizes[MTYPE_PING] - 1); // Store our checksum as the last byte
	
			writePortMsg(ports.xbeePort, "XBee", pingMsg, messageSizes[MTYPE_PING]);  // Write the handshake to the XBee port
			free(pingMsg);

		}

	}

}

/* sendConfig() - Send encoder its configuration */

void sendConfig() {

	unsigned char *configMsg;

	// length: BEGIN + TYPE + LENGTH + 29 + buttonCount + CHECK
	int msgSize = 36 + encoderConfig.buttonCount; 
	configMsg = calloc(msgSize, sizeof(char));

	configMsg[0] = MTYPE_BEGIN;
	configMsg[1] = MTYPE_CONFIG;
	configMsg[2] = msgSize;
	configMsg[3] = encoderConfig.debugPin;                         // Store everything
	configMsg[4] = encoderConfig.statusLEDPin;
	configMsg[5] = encoderConfig.navlightPin;
	configMsg[6] = encoderConfig.rssiPin;
	configMsg[7] = encoderConfig.mainBatteryPin;
	configMsg[8] = encoderConfig.commBatteryPin;
	configMsg[9] = encoderConfig.videoBatteryPin;
	configMsg[10] = encoderConfig.lostMessageThreshold >> 8;       // Shift 8 to the right for high byte
	configMsg[11] = encoderConfig.lostMessageThreshold & 255;      // Strip off anything above 1111 1111
	configMsg[12] = encoderConfig.heartbeatInterval >> 8;          // Shift 8 to the right for high byte
	configMsg[13] = encoderConfig.heartbeatInterval & 255;         // Strip off anything above 1111 1111
	configMsg[14] = encoderConfig.pingInterval >> 8;               // Shift 8 to the right for high byte
	configMsg[15] = encoderConfig.pingInterval & 255;              // Strip off anything above 1111 1111
	configMsg[16] = encoderConfig.servoCount;
	configMsg[17] = encoderConfig.buttonCount;
	configMsg[18] = encoderConfig.ppmMinPulse >> 8;                // Shift 8 to the right for high byte
	configMsg[19] = encoderConfig.ppmMinPulse & 255;               // Strip off anything above 1111 1111
	configMsg[20] = encoderConfig.ppmMaxPulse >> 8;                // Shift 8 to the right for high byte
	configMsg[21] = encoderConfig.ppmMaxPulse & 255;               // Strip off anything above 1111 1111
	configMsg[22] = encoderConfig.ppmHighPulse >> 8;               // Shift 8 to the right for high byte
	configMsg[23] = encoderConfig.ppmHighPulse & 255;              // Strip off anything above 1111 1111
	configMsg[24] = encoderConfig.ppmSyncPulse >> 8;               // Shift 8 to the right for high byte
	configMsg[25] = encoderConfig.ppmSyncPulse & 255;              // Strip off anything above 1111 1111
	configMsg[26] = encoderConfig.statusIntervalSignalLost >> 8;   // Shift 8 to the right for high byte
	configMsg[27] = encoderConfig.statusIntervalSignalLost & 255;  // Strip off anything above 1111 1111
	configMsg[28] = encoderConfig.statusIntervalOK >> 8;           // Shift 8 to the right for high byte
	configMsg[29] = encoderConfig.statusIntervalOK & 255;          // Strip off anything above 1111 1111
	configMsg[30] = encoderConfig.navlightInterval >> 8;           // Shift 8 to the right for high byte
	configMsg[31] = encoderConfig.navlightInterval & 255;          // Strip off anything above 1111 1111
	configMsg[32] = encoderConfig.voltageSamples & 255;
	configMsg[33] = encoderConfig.adcSampleRate >> 8;              // Shift 8 to the right for high byte
	configMsg[34] = encoderConfig.adcSampleRate & 255;             // Strip off anything above 1111 1111

	int x;

	for(x = 0 ; x < encoderConfig.buttonCount ; x++) {

		configMsg[35 + x] = encoderConfig.buttonPinMap[x];

	}

	configMsg[msgSize - 1] = generateChecksum(configMsg, msgSize - 1);
	writePortMsg(ports.xbeePort, "XBee", configMsg, msgSize); // Write out message to XBee
	free(configMsg);
	signalInfo.sentConfig = 1;
	signalInfo.maxdBm = -1000;
	signalInfo.mindBm = 0;

}

/* sendAck(message) - Send ack! */

void sendAck(messageState *msg) {

	unsigned char pingReply[4];

	pingReply[0] = MTYPE_BEGIN;
	pingReply[1] = MTYPE_PING_REPLY;
	pingReply[2] = msg->messageBuffer[2];
	pingReply[3] = generateChecksum(pingReply, messageSizes[MTYPE_PING_REPLY] - 1); // Store our checksum as the last byte

	writePortMsg(ports.xbeePort, "XBee", pingReply, messageSizes[MTYPE_PING_REPLY]);  // Write the handshake to the XBee port
	signalInfo.sentConfig = 1;  // Since we never get a ping before sending config, this is an already configured and running UAV we've connected to

}


/* checkSignal() - Check whether the signal is good, if not RUMBLE!! */

int checkSignal() {

	struct input_event play; // input_event control to play and stop effects
	time_t currentTime = time(NULL); // get current time

	if(signalInfo.handShook) { 

		if(joystickState.rumbleLevel == EFFECTS_COUNT - 1) {
		
			joystickState.rumbleLevel = 0;
		
		}

		double signalTimeDiff = difftime(currentTime, signalInfo.lastMessageTime);

		if(signalTimeDiff > (double)configInfo.timeoutThreshold) {  // Looks like we've lost our signal (>1s since last ping ack)

			signalInfo.handShook = 0;  // Turn off handshake so we can resync
			signalInfo.lostSignalTime = time(NULL); // Store time we lost the signal
			#if DEBUG_LEVEL != 4 && DEBUG_LEVEL != 5
			wprintw(display.outputWindow, "Lost signal!  Attempting to resync.\n");
			wrefresh(display.outputWindow);
			#endif
			return 0;

		}
		
	} else {

		if(signalInfo.initialConnection) {

			double signalTimeDiff = difftime(currentTime, signalInfo.lostSignalTime);
			double rumbleTimeDiff = difftime(currentTime, joystickState.lastRumbleTime);
		
			if(signalTimeDiff > 0.0 && rumbleTimeDiff > 0.5 && joystickState.rumbleLevel < (EFFECTS_COUNT - 1)) {  // Small rumble
	
				joystickState.lastRumbleTime = currentTime;
			
				play.type = EV_FF;
				play.code = joystickState.effects[joystickState.rumbleLevel].id;
				play.value = 1;
				if (write(joystickState.event, (const void*) &play, sizeof(play)) == -1) {
					perror("Play effect");
					exit(1);
				}
				joystickState.rumbleLevel++;
				return 0;
	
			} else if(signalTimeDiff > 0.0 && rumbleTimeDiff > 0.5 && joystickState.rumbleLevel == (EFFECTS_COUNT - 1)) {
	
				joystickState.lastRumbleTime = currentTime;	
				play.type = EV_FF;
				play.code = joystickState.effects[joystickState.rumbleLevel].id;
				play.value = 1;

				if (write(joystickState.event, (const void*) &play, sizeof(play)) == -1) {
					perror("Play effect");
					exit(1);
				}		
				return 0;
		
			}

		}
	
	}
	
	return 1; // Should never get here

}

/* map() - Map a number in inRangeLow->inRangeHigh range into outRangeLow->outRangeHigh */

int map(int value, int inRangeLow, int inRangeHigh, int outRangeLow, int outRangeHigh)
{
	return outRangeLow + (value-inRangeLow)*(outRangeHigh-outRangeLow)/(inRangeHigh-inRangeLow);
}

/* fmap() - Map a number in inRangeLow->inRangeHigh range into outRangeLow->outRangeHigh.. again! */

float fmap(float value, float inRangeLow, float inRangeHigh, float outRangeLow, float outRangeHigh)
{

	float result = outRangeLow + (value-inRangeLow)*(outRangeHigh-outRangeLow)/(inRangeHigh-inRangeLow);
	return result;
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
	char *buffer;

	int rows, cols;
	getmaxyx(display.mainWindow, rows, cols);
	buffer = calloc(cols, sizeof(char));

	int size;

		int x;

		size = snprintf(buffer, cols, " js_crl version %d.%d.%d-%s ", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD, VERSION_TAG);

		wmove(display.mainWindow, 0, 0);

		for(x = 0 ; x < (cols - size) / 2 ; x++) {

			wprintw(display.mainWindow, "-");

		}

		wprintw(display.mainWindow, "%s", buffer);

		for(x = 0 ; x < (cols - size) / 2 ; x++) {

			wprintw(display.mainWindow, "-");

		}
		
		wprintw(display.mainWindow, "\n");

		wprintw(display.mainWindow, "                  ");

		for(x = 0 ; x < BUTTON_COUNT ; x++) {

			wprintw(display.mainWindow, "  [%2d]  ", x);

		}

		wprintw(display.mainWindow, "\nJoystick State:   ");

		for (x = 0; x < joystickState.axes ; x++) {
		
			wprintw(display.mainWindow, "%6d  ", joystickState.axis[x]);  // Print all joystick axes

		}
		wprintw(display.mainWindow, "\n");
		wprintw(display.mainWindow, "Servo State:      ");
		for(x = 0 ; x < SERVO_COUNT ; x++) {

			wprintw(display.mainWindow, "  %4d  ", airframeState.servos[x] + 1);  // Print all airframe servos

		}
		wprintw(display.mainWindow, "\n");
		wprintw(display.mainWindow, "Button State:   ");
		for(x = 0 ; x < BUTTON_COUNT ; x++) {

			wprintw(display.mainWindow, "      %2d", airframeState.buttons[x]);  // Print all button states
	
		}
		wprintw(display.mainWindow, "\n");
		for(x = 0 ; x < cols ; x++) {

			wprintw(display.mainWindow, "-");

		}
		wprintw(display.mainWindow, "\n");		

		// quick & dirty for now	 DEBUG

		float gcsCommBattery = fmap((float)relay.commBatteryVoltage, 0.0, 1023.0, 0.0, 4.2 * 2.0); // 2S battery
		float uavCommBattery = fmap((float)airframeState.commBatteryVoltage, 0.0, 1023.0, 0.0, 4.2 * 2.0);
		float uavVideoBattery = fmap((float)airframeState.videoBatteryVoltage, 0.0, 1023.0, 0.0, 4.2 * 3.0); // 3S battery
		float gcsVideoBattery = fmap((float)relay.videoBatteryVoltage, 0.0, 1023.0, 0.0, 4.2 * 3.0);
		float uavMainBattery = fmap((float)airframeState.mainBatteryVoltage, 0.0, 1023.0, 0.0, 4.2 * 3.0);

		if(gcsCommBattery < 3.7 * 2.0) { gcsCommBattery = 3.7 * 2.0; }
		if(uavCommBattery < 3.7 * 2.0) { uavCommBattery = 3.7 * 2.0; }
		if(uavVideoBattery < 3.7 * 3.0) { uavVideoBattery = 3.7 * 3.0; }
		if(gcsVideoBattery < 3.7 * 3.0) { gcsVideoBattery = 3.7 * 3.0; }
		if(uavMainBattery < 3.7 * 3.0) { uavMainBattery = 3.7 * 3.0; }

		float gcsCommPercent = fmap(gcsCommBattery, 3.7 * 2.0, 4.2 * 2.0, 0.0, 100.0);
		float uavCommPercent = fmap(uavCommBattery, 3.7 * 2.0, 4.2 * 2.0, 0.0, 100.0);
		float uavVideoPercent = fmap(uavVideoBattery, 3.7 * 3.0, 4.2 * 3.0, 0.0, 100.0);
		float gcsVideoPercent = fmap(gcsVideoBattery, 3.7 * 3.0, 4.2 * 3.0, 0.0, 100.0);
		float uavMainPercent = fmap(uavMainBattery, 3.7 * 3.0, 4.2 * 3.0, 0.0, 100.0);

		char *localBars = calloc(5, sizeof(char));
		char *remoteBars = calloc(5, sizeof(char));
		char *videoBars = calloc(5, sizeof(char));

		rssiBars(signalInfo.localdBm, localBars);
		rssiBars(signalInfo.remotedBm, remoteBars);
		rssiBars(signalInfo.videodBm, videoBars);				

		wprintw(display.mainWindow, "GCS RSSI:         %4d dBm [%s]                    videoRx RSSI:  %4d dBm [%s]\n", signalInfo.localdBm, localBars, signalInfo.videoRSSI, videoBars);
		wprintw(display.mainWindow, "GCS Comm Battery: %4d (%5.3f%% - %5.2fV)  Video Battery:  %4d (%5.3f%% - %5.2fV)\n", relay.commBatteryVoltage, gcsCommPercent, 
									gcsCommBattery, relay.videoBatteryVoltage, gcsVideoPercent, gcsVideoBattery);

		wprintw(display.mainWindow, "\nUAV RSSI:         %4d dBm [%s]\n", signalInfo.remotedBm, remoteBars);

		wprintw(display.mainWindow, "UAV Main Battery: %4d (%5.3f%% - %5.2fV)   Comm Battery:  %4d (%5.3f%% - %5.2fV)  Video Battery: %4d (%5.3f%% - %5.2fV)\n", airframeState.mainBatteryVoltage, 
									uavMainPercent, uavMainBattery, airframeState.commBatteryVoltage, uavCommPercent, uavCommBattery, 
									airframeState.videoBatteryVoltage, uavVideoPercent, uavVideoBattery);

		wprintw(display.mainWindow, "Max: %4d Min: %4d\n", signalInfo.maxdBm, signalInfo.mindBm);

		free(localBars);
		free(remoteBars);
		free(videoBars);
		free(buffer);

		double diff = difftime(currentTime, startTime);

		if(diff < (double)configInfo.throttleSafety) {

			mvwprintw(display.mainWindow, 1, 0, "Throttle Safety Remaining: %3.0f", (double)configInfo.throttleSafety - diff);

		}

		wrefresh(display.mainWindow);
		wrefresh(display.border1);
		wrefresh(display.outputWindow);
		wrefresh(display.border2);
		wrefresh(display.debugWindow);
		wrefresh(display.border3);
		wrefresh(display.errorWindow);

}

void rssiBars(int rssi, char *buffer) {

	if(rssi <= -100) {

		strcpy(buffer, "     ");

	} else if(rssi < -100) {

		strcpy(buffer, "#    ");

	} else if(rssi < -94) {

		strcpy(buffer, "##   ");

	} else if(rssi < -88) {

		strcpy(buffer, "###  ");

	} else if(rssi < -82) {

		strcpy(buffer, "#### ");

	} else if(rssi < -53) {

		strcpy(buffer, "#####");

	} else {

		strcpy(buffer, " wat ");

	}

}
