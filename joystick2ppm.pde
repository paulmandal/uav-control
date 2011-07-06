
/* joystick2ppm - Paul Mandal (paul.mandal@gmail.com)
 * 3.0 - Recieves encoded servo messages from joystick app on GCS
 *     - Updates PPM pulses at 100Hz
 *     - Produces PPM frame every 20ms
 *     - Relays non-servo USART messages to second USART
 *     - Relays second USART (PPZ) messages to USART / GCS
 *     - Uses timers and cool stuff like that
 *     - Has pin mapping for joystick buttons
 * 
 * Thanks to the Arduino community, everyone on the Sanguino team, and everyone involved in science and maths and them things.
 * And fanks to my friend Liz for showing me that science ain't just for nerdy blokes with pocket protectas and spectacles, but can also be quite a laugh.
 *
 * For reference this is coded for the ATmega644P, it has these pins: 
 * PWM: 3, 4, 12, 13, 14, 15
 * Digital I/O: 0, 1, 2, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23
 * USART: 8, 9, 10, 11
 * ADC: 24, 25, 26, 27, 28, 29 (I think?)
 *
 * There is an option to use this with an ATmega328P in a more limited capacity (e.g. for testing)
 */

/* Including things */

#include "WProgram.h" 
#include "joystickRC_structs.h"

/* This is the defining moment of the file */

#define DEBUG_LEVEL 0            // 1 - Messaging debugging
                                 // 2 - Servo / pin output
                                 // 3 - Signal continuity debugging (light 4 stays on if signal is ever lost)
                                 // 4 - Signal continuity (with serial output)
                                 // 5 - PPM registers
                                 // 6 - PPM pulse values
                                 // 7 - Only start debug message
                                 // 8 - Report bad checksums
			         // 9 - Report good and bad checksums
		           	 // 10 - Processing debug

#define VERSION_MAJOR          3 // Major version #
#define VERSION_MINOR          2 // Minor #
#define VERSION_MOD            1 // Mod #
#define VERSION_TAG        "DBG" // Tag

#define FLASHING_LIGHTS        2 // Number of flashing lights our board has

#define MSG_BUFFER_SIZE      256 // Message buffer size in bytes

#define PPZ_MSG_HEADER_SIZE    3 // PPZ msg header size in bytes

/* Important Numbers */

typedef enum _messageTypes {

	MTYPE_BEGIN = 0,
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
	MTYPE_CONFIG

} messageTypes;

typedef enum _lightPins {

	NAV_LIGHT = 0,
	STATUS_LIGHT

} lightPins;

byte messageSizes[] = {1, 4, 4, 3, 22, 5, -1, 19, 6, -1, -1, -1, 7, -1};

/* Various varibles to hold state info */

unsigned int *servos;       // store servo states
unsigned int *buttons;      // store button states

ledBlinker lights[FLASHING_LIGHTS];  // blinking lights state

configValues configInfo;   // Configuration
ppmState      ppmInfo;     // PPM state info
signalState   signalInfo;  // Signal state info

messageState xbeeMsg;  // Message struct for messages from XBee line
#ifdef __AVR_ATmega644P__
messageState ppzMsg;  // Message struct for messages from PPZ line
#endif
#if DEBUG_LEVEL > 0
messageState dbgMsg;  // Message struct for outgoing debug messages
#endif

#ifdef __AVR_ATmega644P__

#endif

/* Setup function */

void setup() {

	randomSeed(analogRead(0));          // Seed our random number gen with an unconnected pin's static
	initSignal();
	initLights();
	initMessage(&xbeeMsg);              // Init our XBee message
	#ifdef __AVR_ATmega644P__
	initMessage(&ppzMsg);               // Init our PPZ message
	ppzMsg.readBytes = PPZ_MSG_HEADER_SIZE; // Leave room for header addition to PPZ message
	ppzMsg.length = PPZ_MSG_HEADER_SIZE;    // Leave room for header addition to PPZ message
	#endif
	initTimer();                        // Init our timer
	Serial.begin(115200);               // Open XBee/GCS Serial
	Serial.flush();
	#ifdef __AVR_ATmega644P__
	Serial1.begin(115200);              // Open PPZ Serial
	Serial1.flush();
	#endif
  
	#if DEBUG_LEVEL > 0
	initMessage(&dbgMsg);      // Init our debug message
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---joystick2ppm version %d.%d.%d-%s... Open for debugging mode...-", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD, VERSION_TAG);  // Write a debug message leading and trailing dashes will be replaced with header and checksums
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);  // Send debug message
	#endif
  
	#if DEBUG_LEVEL == 3 || DEBUG_LEVEL == 4
	pinMode(DEBUG_PIN1, OUTPUT);  //  DEBUG - Pin will light permanently if signal is lost
	#endif

}

/* Loop function */

void loop() {

	byte x;
	// keep track of last time we send a heartbeat
	updateLights();        // Check if we need to update any lights

	for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) { // checkMessage functions should be run with a much higher frequency than the LED updates or handleSignal()
  
		checkXBeeMessages(&xbeeMsg); // Check for incoming XBee messages
		#ifdef __AVR_ATmega644P__
		checkPPZMessages(&ppzMsg);   // Check for incoming PPZ messages
		#endif

	}

	handleSignal();  // Check if the signal is still good

}

/* Function definitions */

/* initSignal() - Initialise signal info */

void initSignal() {

	signalInfo.handShook = false;
	signalInfo.firstSignalEstablished = false;
	signalInfo.pingData = 0;
	signalInfo.lastMessageTime = -1000UL; // Time of last legit message, -100 initially so the PPM won't turn on until we get a real message
	signalInfo.lastMessageSentTime = 0UL;
	signalInfo.ctrlCounter = 0;

}

/* freeMemory() - Free up memory */

void freeMemory() {

	free(servos);
	free(buttons);
	free(configInfo.buttonPinMap);
	free(ppmInfo.pulses);

}

/* initControlState() - Zeroes out everythang */

void initControlState() {

	// Allocate memory for servos and buttons

	servos = (unsigned int*)calloc(configInfo.servoCount, sizeof(int));
	buttons = (unsigned int*)calloc(configInfo.buttonCount, sizeof(int));

	byte x;

  	// Zero out all buttons and servos

	for(x = 0 ; x < configInfo.servoCount ; x++) {

		servos[x] = 0;

	}

	for(x = 0 ; x < configInfo.buttonCount ; x++) {

		buttons[x] = 0;

	}

}

/* initPPM - What do you think? */

void initPPM() {

	byte x;
	int midPPMPulse = (configInfo.ppmMinPulse + configInfo.ppmMaxPulse) / 2;  
	ppmInfo.pulses = (int*)calloc(configInfo.ppmPulses, sizeof(int));

	for (x = 0 ; x < (configInfo.servoCount + 1) ; x++) {
    
		ppmInfo.pulses[x * 2] = configInfo.ppmHighPulse;
		ppmInfo.pulses[(x * 2) + 1] = midPPMPulse;       // Set all PPM pulses to halfpulse
    
	}

	ppmInfo.pulses[configInfo.ppmPulses - 1] = configInfo.ppmSyncPulse; // Sync pulse is before 0 length pulse

	#if DEBUG_LEVEL == 5
	for(x = 0 ; x < configInfo.ppmPulses ; x++) {
   
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Pulse[%d]: %d -", x, pulses[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                              // Write debug message
    
	}
	#endif
	ppmInfo.currentPulse = 0; // init currentPulse
	ppmInfo.ppmON = false;

}

/* initLights - Set up lights */

void initLights () {

	byte x;        

        for(x = 0 ; x < FLASHING_LIGHTS ; x++) {
  
		lights[x].interval = -1; // Turn light off
		lights[x].pin = -1;  // Disable light pin

        }

}

/* initOutputs() - Set output pins up */

void initOutputs() {

  	byte x;
  	
  	for(x = 0 ; x < configInfo.buttonCount ; x++) {
  	
  		if(configInfo.buttonPinMap[x] > 0 && configInfo.buttonPinMap[x] < 128) {
  		
  			pinMode(configInfo.buttonPinMap[x], OUTPUT);
  		
  		}
  	
  	}
  
        lights[NAV_LIGHT].pin = configInfo.navlightPin;
        lights[NAV_LIGHT].state = false;
        lights[NAV_LIGHT].lastChanged = 0;
        lights[NAV_LIGHT].interval = -1;
        
        lights[STATUS_LIGHT].pin = configInfo.statusLEDPin;
        lights[STATUS_LIGHT].state = false;
        lights[STATUS_LIGHT].lastChanged = 0;
        lights[STATUS_LIGHT].interval = configInfo.statusIntervalSignalLost;

	for(x = 0 ; x < FLASHING_LIGHTS ; x++) {

		pinMode(lights[x].pin, OUTPUT);

	}
 
}

/* initMessage() - Initialise message */

boolean initMessage(messageState *msg) {

        byte x;
	msg->readBytes = 0;
	msg->length = -1; // Init message.length as header length size
	if((msg->messageBuffer = (unsigned char*)calloc(MSG_BUFFER_SIZE, sizeof(char))) != NULL) {

		return true; // calloc() worked
	
	} else {
	
		return false; // calloc() failed
	
	} 
	
}

/* checkXBeeMessages() - Check for and handle any incoming messages */

boolean checkXBeeMessages(messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(msg->length == -1) { // Message has < 0 length, check if anything in messageBuffer can fill that in

		msg->length = getMessageLength(msg);

	}

	if(msg->readBytes < msg->length || msg->length == -1) {  // We either aren't done reading the message or we don't have MTYPE_BEGIN and/or MTYPE and/or PARAM to tell us the real length

		if(Serial.available() > 0)  {  // Byte is availabe, read it

			testByte = Serial.read();

			if(msg->readBytes == 0) { // Haven't got MTYPE_BEGIN yet, look for it

				if(testByte == MTYPE_BEGIN) { // Beginning of a messge

					msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
					msg->readBytes++;			       // Increment readBytes

				}

			} else {

				msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
				msg->readBytes++;			       // Increment readBytes

			}

			return true;

		} else {

 			return false;

		}	

	} else { 

		if(msg->length > 0) { // Message is finished, process it

			if(testChecksum(msg->messageBuffer, msg->length)) { // Checksum passed, process message..  

				processMessage(msg);
				if(msg->messageBuffer[1] != MTYPE_PING) {
										
					signalInfo.lastMessageTime = millis(); // Set last message time, except for from a ping

				}

			} 

		}

		byte x;

		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

			msg->messageBuffer[x] = '\0';

		}

		msg->readBytes = 0;   // Zero out readBytes
		msg->length = -1;     // Set message length to -1
		
		return true;

	}

}

/* checkPPZMessages() - Check for and handle any incoming PPZ messages */

#ifdef __AVR_ATmega644P__
boolean checkPPZMessages(messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(Serial1.available() > 0) {

		testByte = Serial1.read();  // Read our byte		

		#if DEBUG_LEVEL == 1
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---PPZBYTE[%d - CSLA: %d]: %x-", dbgMsg.readBytes, commandsSinceLastAck, testByte);
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif		

		msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
		msg->readBytes++;			       // Increment readBytes
                msg->length++;                                 // Increment length
		
		if(testByte == '\n') { // This is the message end, relay the message to GCS and reset msg
		
			writeXBeeMessage(msg, MTYPE_PPZ);

			msg->readBytes = PPZ_MSG_HEADER_SIZE;  // Leave room for header to be added
                        msg->length = PPZ_MSG_HEADER_SIZE;
			byte x;	

			// Clear out message so it's ready to be used again	
			for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

				msg->messageBuffer[x] = '\0';

			}
		
		}
	
		return true;
	
	} else {

		return false;
	
	}

}
#endif

/* getMessageLength(msg) */

int getMessageLength(messageState *msg) {

	if(msg->readBytes == 2) { // Do zero-parameter types first, if we can't find one, see if we have enough characters for one of the parametered types
		
		byte size = messageSizes[msg->messageBuffer[1]];

		return -1; // Size will be the message size or -1 for parametered types

	} else if(msg->readBytes > 2) { // Didn't find any non-parameter message types, let's see if we have a parametered one

		if(msg->messageBuffer[1] == MTYPE_PPZ || msg->messageBuffer[1] == MTYPE_DEBUG) {

			byte msgLength = msg->messageBuffer[2];
			if(msgLength < MSG_BUFFER_SIZE) {
	
				return msgLength;  // PPZ & Debug messages have length as param

			} else {

				return -2; // Bogus message

			}

		} else if(msg->messageBuffer[1] == MTYPE_VAR_SERVOS) {
		
			byte numServos = msg->messageBuffer[2];

			if(numServos < configInfo.servoCount) {  // If we get close to configInfo.servoCount the sent message would be a ALL_SERVOS or FULL_UPDATE

				return 4 + numServos * 2;  // Convert servo count to # of bytes (2 bytes per servo + begin + type + param + check)

			} else {

				return -2; // Bogus message
			}

		} else if(msg->messageBuffer[1] == MTYPE_CONFIG) {

			byte msgLength = msg->messageBuffer[2];
			if(msgLength < MSG_BUFFER_SIZE) {

				return msgLength; 

			} else {

				return -2; // We shouldn't get messages larger than we can handle

			}

		} else {

			return -2; // No valid message types to provide length found

		}

	} else {

		return -1; // Haven't read enough bytes yet

	}

}

boolean readConfig(messageState *msg) {

	// Plunder the message for values

     	configInfo.debugPin                 = msg->messageBuffer[3];		                             	                  // Pin for debug LED
	configInfo.statusLEDPin             = msg->messageBuffer[4];          	                               	                  // Pin for status LED
	configInfo.navlightPin              = msg->messageBuffer[5];                                         	 	          // Pin for navlight LED/LEDs
	configInfo.rssiPin                  = msg->messageBuffer[6];                                          	    	          // RSSI input pin
	configInfo.mainBatteryPin           = msg->messageBuffer[7];          	                             	                  // Main battery input pin
	configInfo.commBatteryPin           = msg->messageBuffer[8];                                      		          // Comm battery input pin (this board)
	configInfo.videoBatteryPin          = msg->messageBuffer[9];                                      		          // Video battery input pin

	configInfo.lostMessageThreshold     = 0;			 			                                  // Time in ms without a message before assuming we've lost our signal
	configInfo.lostMessageThreshold     = msg->messageBuffer[10] << 8;                                                        // Shift 8 to the left for the high byte     
	configInfo.lostMessageThreshold     = configInfo.lostMessageThreshold | msg->messageBuffer[11];                           // OR with the low byte

	configInfo.heartbeatInterval        = 0;        					                                  // Interval in ms to send our heartbeat
	configInfo.heartbeatInterval        = msg->messageBuffer[12] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.heartbeatInterval        = configInfo.heartbeatInterval | msg->messageBuffer[13];                              // OR with the low byte

	configInfo.pingInterval             = 0;                                                                                  // Interval in ms to send pings
	configInfo.pingInterval             = msg->messageBuffer[14] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.pingInterval             = configInfo.pingInterval | msg->messageBuffer[15];                                   // OR with the low byte

	configInfo.servoCount               = msg->messageBuffer[16];                                                             // # of servos on this board
	configInfo.buttonCount              = msg->messageBuffer[17];                                                             // # of buttons on this board

	configInfo.ppmPulses 		    = ((configInfo.servoCount * 2) + 2);                                                  // How many pulses are there in the whole PPM

   	configInfo.ppmMinPulse              = 0;                                                                                  // PPM min pulse length in 1/2 usec (default = 2000 = 1ms)
	configInfo.ppmMinPulse              = msg->messageBuffer[18] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.ppmMinPulse              = configInfo.ppmMinPulse | msg->messageBuffer[19];                                    // OR with the low byte

   	configInfo.ppmMaxPulse              = 0;                                                                                  // PPM max pulse length in 1/2 usec (default = 4000 = 2ms)
	configInfo.ppmMaxPulse              = msg->messageBuffer[20] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.ppmMaxPulse              = configInfo.ppmMaxPulse | msg->messageBuffer[21];                                    // OR with the low byte

	configInfo.ppmHighPulse             = 0;                                                                                  // PPM high pulse duration in 1/2 usec (default = 400 = 200usec)
	configInfo.ppmHighPulse             = msg->messageBuffer[22] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.ppmHighPulse             = configInfo.ppmHighPulse | msg->messageBuffer[23];                                   // OR with the low byte

	configInfo.ppmSyncPulse             = 0;                                                                                  // PPM sync pulse duration in 1/2 usec (default = 8000 = 4ms)
	configInfo.ppmSyncPulse             = msg->messageBuffer[24] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.ppmSyncPulse             = configInfo.ppmSyncPulse | msg->messageBuffer[25];                                   // OR with the low byte

	configInfo.statusIntervalSignalLost = 0;                                                                                  // Interval for status to flash when signal is lost
	configInfo.statusIntervalSignalLost = msg->messageBuffer[26] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.statusIntervalSignalLost = configInfo.statusIntervalSignalLost | msg->messageBuffer[27];                       // OR with the low byte

	configInfo.statusIntervalOK         = 0;                                                                                  // Interval for status to flash when everything is OK
	configInfo.statusIntervalOK         = msg->messageBuffer[28] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.statusIntervalOK         = configInfo.statusIntervalOK | msg->messageBuffer[29];                               // OR with the low byte

	configInfo.navlightInterval         = 0;                                                                                  // Interval for navlights to flash
	configInfo.navlightInterval         = msg->messageBuffer[30] << 8;                                                        // Shift 8 to the left for the high byte
	configInfo.navlightInterval         = configInfo.navlightInterval | msg->messageBuffer[31];                               // OR with the low byte

	configInfo.buttonPinMap		    = (byte*)calloc(configInfo.buttonCount, sizeof(byte));	               	          // Allocate memory for the button pin map

	byte x;

	for(x = 0 ; x < configInfo.buttonCount ; x++) {

		configInfo.buttonPinMap[x] = msg->messageBuffer[32 + x];

	}

	return true;

}

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(messageState *msg) {

        byte x;
	unsigned char msgType = msg->messageBuffer[1];

	if(msgType == MTYPE_PING) { // We got a ping, send an ack

		sendAck(msg);

	} else if(msgType == MTYPE_PING_REPLY) {  // Handle the message, since it got past checksum it has to be legit

		if(msg->messageBuffer[2] == signalInfo.pingData) { //  See if the payload matches the ping packet we sent out
		
			signalInfo.handShook = true;

		}

	} else if(msgType == MTYPE_CONFIG) {  // Configuration

		if(readConfig(msg)) {

			if(signalInfo.firstSignalEstablished) {  // We've already been connected and received a config, free up the memory that config took

				// free up memory
				freeMemory();

			}

			initControlState();                        // Initialise control state
			initOutputs();                             // Initialise outputs
			initPPM();                                 // Set default PPM pulses
			signalInfo.firstSignalEstablished = true; // Got config info, we're set up with the ground station, send our ping out to complete handshake

		}

	} else if(msgType == MTYPE_SINGLE_SERVO) {

		byte servoNum = 0;
		int servoPos = 0;

		servoNum = (msg->messageBuffer[2] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
		servoPos = (msg->messageBuffer[2] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
		servoPos = servoPos | msg->messageBuffer[3]; // Store last 8 bits of servo position
		storePulse(servoNum, servoPos, 0, 1023);
		servos[servoNum] = servoPos;

		#if DEBUG_LEVEL == 10
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updated Servo[%d] to pos: %d-", servoNum, servoPos);
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif	

	} else if(msgType == MTYPE_VAR_SERVOS) {

		byte numServos = 0;
		byte servoNum = 0;
		int servoPos = 0;

		numServos = msg->messageBuffer[2];

		#if DEBUG_LEVEL == 10
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updating [%d] servos-", numServos);
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif	

		for(x = 0 ; x < numServos ; x++) {

			servoNum = (msg->messageBuffer[(x * 2) + 3] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
			servoPos = (msg->messageBuffer[(x * 2) + 3] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
			servoPos = servoPos | msg->messageBuffer[(x * 2) + 4]; // Store last 8 bits of servo position
			storePulse(servoNum, servoPos, 0, 1023);
			servos[servoNum] = servoPos;

			#if DEBUG_LEVEL == 10
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updated Servo[%d] to pos: %d-", servoNum, servoPos);
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
			#endif	

		}

	} else if(msgType == MTYPE_ALL_SERVOS) {

		byte servoNum = 0;
		int servoPos = 0;

		#if DEBUG_LEVEL == 10
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updating all servos-");
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif	

		for(x = 0 ; x < configInfo.servoCount ; x++) {

			servoNum = (msg->messageBuffer[(x * 2) + 2] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
			servoPos = (msg->messageBuffer[(x * 2) + 2] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
			servoPos = servoPos | msg->messageBuffer[(x * 2) + 3]; // Store last 8 bits of servo position
			storePulse(servoNum, servoPos, 0, 1023);
			servos[servoNum] = servoPos;

			#if DEBUG_LEVEL == 10
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updated Servo[%d] to pos: %d-", servoNum, servoPos);
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
			#endif	

		}

	} else if(msgType == MTYPE_FULL_UPDATE) {

		byte servoNum = 0;
		int servoPos = 0;

		#if DEBUG_LEVEL == 10
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updating all servos-");
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif	

		for(x = 0 ; x < configInfo.servoCount ; x++) {

			servoNum = (msg->messageBuffer[(x * 2) + 2] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
			servoPos = (msg->messageBuffer[(x * 2) + 2] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
			servoPos = servoPos | msg->messageBuffer[(x * 2) + 3]; // Store last 8 bits of servo position
			storePulse(servoNum, servoPos, 0, 1023);
			servos[servoNum] = servoPos;

			#if DEBUG_LEVEL == 10
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updated Servo[%d] to pos: %d-", servoNum, servoPos);
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
			#endif	

		}

		for(x = 0 ; x < 3 ; x++) {  // This loop handles 4 buttons at once since each uses 2 bits and we read in 1 byte (2 bits * 4 = 8 bits = 1 byte)

			buttons[(x * 4)] = (msg->messageBuffer[x + (2 + (configInfo.servoCount * 2))] & B11000000) >> 6;     // Bitwise and against our byte to strip away other button values, then bitshift to 0th and 1st positions
			buttons[(x * 4) + 1] = (msg->messageBuffer[x + (2 + (configInfo.servoCount * 2))] & B00110000) >> 4; // Same, you can see the bitmask shift to the right as we work out way down the byte
			buttons[(x * 4) + 2] = (msg->messageBuffer[x + (2 + (configInfo.servoCount * 2))] & B00001100) >> 2; // Same
			buttons[(x * 4) + 3] = (msg->messageBuffer[x + (2 + (configInfo.servoCount * 2))] & B00000011);      // No bitshift here since our bits are already in 0th and 1st pos.

	        }

		#if DEBUG_LEVEL == 10
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updated buttons-");
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif	

		handleButtonUpdate();

	} else if(msgType == MTYPE_BUTTON_UPDATE) {

		for(x = 0 ; x < 3 ; x++) {  // This loop handles 4 buttons at once since each uses 2 bits and we read in 1 byte (2 bits * 4 = 8 bits = 1 byte)

			buttons[(x * 4)] = (msg->messageBuffer[x + 2] & B11000000) >> 6;     // Bitwise and against our byte to strip away other button values, then bitshift to 0th and 1st positions
			buttons[(x * 4) + 1] = (msg->messageBuffer[x + 2] & B00110000) >> 4; // Same, you can see the bitmask shift to the right as we work out way down the byte
			buttons[(x * 4) + 2] = (msg->messageBuffer[x + 2] & B00001100) >> 2; // Same
			buttons[(x * 4) + 3] = (msg->messageBuffer[x + 2] & B00000011);      // No bitshift here since our bits are already in 0th and 1st pos.

	        }

		#if DEBUG_LEVEL == 10
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Updated buttons-");
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif	

		handleButtonUpdate();

	} else if(msgType == MTYPE_PPZ) { // Handle PPZ message
	
		#ifdef __AVR_ATmega644P__
		writePPZMessage(msg);
		#endif
	
	}

}

/* generateChecksum(message, length) - Generate a checksum for message */

unsigned char generateChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	byte x;

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x]; // Generate checksum

	}

	return checksum;

}

/* testChecksum(message, length) - Test if the last byte checksum is good */

int testChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	byte x;

	#if DEBUG_LEVEL == 1
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---CHKMSG:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message
	#endif

	for(x = 0 ; x < length ; x++) {

		#if DEBUG_LEVEL == 1
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---%x-", (unsigned int)message[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message
		#endif
                checksum = checksum ^ (unsigned int)message[x];  // Test this message against its checksum (last byte)

	}
	#if DEBUG_LEVEL == 1
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---CHK: %x-", checksum); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message
	#endif

	if(checksum == 0x00) {

		#if DEBUG_LEVEL == 9
		if(message[1] != MTYPE_HEARTBEAT && message[1] != MTYPE_PING) {

			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Good checksum (length: %d, type: %d): %2x-", length, message[1], checksum); // Build debug message
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message		
		}
		#endif
		return true;  // Checksum passed!

	} else {

		#if DEBUG_LEVEL == 8 || DEBUG_LEVEL == 9
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Bad checksum (length: %d, type: %d): %2x-", length, message[1], checksum); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message		
		#endif
		return false;

	}

}

/* sendHeartbeat() - Send heartbeat */

void sendHeartbeat() {

	unsigned char *heartbeat;

	heartbeat = (unsigned char*)calloc(messageSizes[MTYPE_HEARTBEAT], sizeof(char));
        #if DEBUG_LEVEL == 1
      	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Sending HEARTBEAT:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                   // Write debug message
	#endif 

	heartbeat[0] = MTYPE_BEGIN;
	heartbeat[1] = MTYPE_HEARTBEAT;
	heartbeat[2] = generateChecksum(heartbeat, messageSizes[MTYPE_HEARTBEAT] - 1); // Store our checksum as the last byte
	
	Serial.write(heartbeat, messageSizes[MTYPE_HEARTBEAT]);     // Send the sync ACK
	signalInfo.lastMessageSentTime = millis();
	free(heartbeat);

}

/* sendStatus() - Send status */

void sendStatus() {

	unsigned char *status;
	int mainVoltage = 0;
	int commVoltage = 0;
	int videoVoltage = 0;
	int rssi = 0;

	status = (unsigned char*)calloc(messageSizes[MTYPE_STATUS], sizeof(char));

	rssi = analogRead(configInfo.rssiPin);
	rssi = map(rssi, 0, 1023, 0, 255);
	mainVoltage = analogRead(configInfo.mainBatteryPin);
	mainVoltage = map(mainVoltage, 0, 1023, 0, 255);
	commVoltage = analogRead(configInfo.commBatteryPin);
	commVoltage = map(commVoltage, 0, 1023, 0, 255);
	videoVoltage = analogRead(configInfo.videoBatteryPin);
	videoVoltage = map(videoVoltage, 0, 1023, 0, 255);

	status[0] = MTYPE_BEGIN;
	status[1] = MTYPE_STATUS;
	status[2] = rssi;
	status[3] = mainVoltage;
	status[4] = commVoltage;
	status[5] = videoVoltage;
	status[6] = generateChecksum(status, messageSizes[MTYPE_STATUS] - 1); // Store our checksum as the last byte
	
	Serial.write(status, messageSizes[MTYPE_STATUS]);     // Send the status
	signalInfo.lastMessageSentTime = millis();
	free(status);

}

/* sendPing() - Send ping! */

void sendPing() {

	unsigned char ping[4];
	signalInfo.pingData = random(0, 256);

	ping[0] = MTYPE_BEGIN;
	ping[1] = MTYPE_PING;
	ping[2] = signalInfo.pingData;
	ping[3] = generateChecksum(ping, messageSizes[MTYPE_PING] - 1); // Store our checksum as the last byte

	Serial.write(ping, messageSizes[MTYPE_PING]);     // Send the ping
	signalInfo.lastMessageSentTime = millis();

}

/* sendAck(message) - Send ack! */

void sendAck(messageState *msg) {

	unsigned char pingReply[4];

        #if DEBUG_LEVEL == 1
      	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Sending PING ACK:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                   // Write debug message
	#endif 

	pingReply[0] = MTYPE_BEGIN;
	pingReply[1] = MTYPE_PING_REPLY;
	pingReply[2] = msg->messageBuffer[2];
	pingReply[3] = generateChecksum(pingReply, messageSizes[MTYPE_PING_REPLY] - 1); // Store our checksum as the last byte

	Serial.write(pingReply, messageSizes[MTYPE_PING_REPLY]);     // Send the sync ACK
	signalInfo.lastMessageSentTime = millis();

}

/* updateLights() - Update lights based on things */

void updateLights() {

	byte x;
        
	for(x = 0 ; x < FLASHING_LIGHTS ; x++) {
          
		if(lights[x].pin > 0) {

			if(lights[x].interval > 0) {

				unsigned long currentTime = millis(); // get current time
				if(currentTime - lights[x].lastChanged > lights[x].interval) {
                
                		digitalWrite(lights[x].pin, lights[x].state);
                		lights[x].state = !lights[x].state;
                		lights[x].lastChanged = currentTime;
              
       	      			}

			} else if(lights[x].interval == 0) {
                            
				digitalWrite(lights[x].pin, HIGH);
              
	     		} else {	
	
				digitalWrite(lights[x].pin, LOW);	

	          	}

		}
          
        }
	
}

/* handleSignal() - Check the signal state and make necessary updates */

void handleSignal() {

	unsigned long currentTime = millis(); // get current time
	if(signalInfo.handShook) { // Signal is still good last we checked

		if((currentTime - signalInfo.lastMessageTime) > configInfo.lostMessageThreshold) { // Check if the signal is actually still good
     
			cli(); // Do not allow timer ppm disabling to be interrupted
			signalInfo.handShook = false;                               // If we haven't received a message in > LOST_MSG_THRESHOLD set handShook = false
			ppmInfo.ppmON = false;                                   // Disable PPM
			TIMSK1 = B00000000;                              // Disable interrupt on compare match
			TCCR1A = B00000000;                              // Disable fast PWM     
			TCCR1B = B00000000;                              // Disable fast PWM, clock, and prescaler
      
			TCCR1A = B10000000;                              // Set the pin to go low on compare match
			TCCR1C = B10000000;                              // Force match, this will force the pin low
			#ifdef __AVR_ATmega644P__
			DDRD  &= B11011111;                              // Disable output on OC1A      
			#else
			DDRB  &= B11111101;                              // Disable output on OC1A
			#endif
			#if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 4
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Stopping PPM, handShook = false-"); // Build debug message
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                               // Write debug message
			#endif
			#if DEBUG_LEVEL == 4
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---currentTime: %lu lastMessageTime: %lu diff: %lu > %lu-", currentTime, lastMessageTime, (currentTime - lastMessageTime), LOST_MSG_THRESHOLD); // Build debug message
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                               // Write debug message
			#endif
			lights[STATUS_LIGHT].interval = configInfo.statusIntervalSignalLost; // Set status LED interval to signal lost
			#if DEBUG_LEVEL == 3 || DEBUG_LEVEL == 4
			digitalWrite(DEBUG_PIN1, HIGH);
			#endif
			sei(); // Re-enable interrupts

		} else { // The signal is good, do we need to send a heartbeat?

			if((currentTime - signalInfo.lastMessageSentTime) > configInfo.heartbeatInterval) {

				if(signalInfo.ctrlCounter % 3 == 0) { // Send a status message instead of every 3rd heartbeat
	
					signalInfo.ctrlCounter = 0;
					sendStatus();

				} else {

					sendHeartbeat();

				}

				signalInfo.ctrlCounter++;

			}

			if(!ppmInfo.ppmON) {  // Restart PPM since it was off

				cli();  // This shouldn't get interrupted since PPM is off but just to be safe..

				ppmInfo.ppmON = true;                           // turn on PPM status flag
				lights[STATUS_LIGHT].interval = configInfo.statusIntervalOK; // Set our status LED interval to OK

				#if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 4
				dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Starting PPM, lostSignal = false-"); // Build debug message
				writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                               // Write debug message
				#endif        
		        
				TCCR1B = B00001000;                     // CTC mode, clock disabled, OCR1A will never be reached by TCNT1 'coz no clock is running
				TCCR1A = B11000000;                     // CTC, set OC1A HIGH on match
	
				OCR1A = 0xFFFF;                         // Make OCR1A max so it doesn't get hit
	
				TCCR1C = B10000000;                     // Force match, should set pin high, WILL NOT generate ISR() call        

				OCR1A = ppmInfo.pulses[0];                      // Set OCR1A to pulse[0], this won't actually matter until we set TCCR1A and TCCR1B at the end to enable fast PWM
				ppmInfo.currentPulse = 1;              // Set currentPulse to 1 since there will be no ISR() call to increment it

				#ifdef __AVR_ATmega644P__
				DDRD  |= B00100000;                     // Enable output on OC1A
				#else
				DDRB  |= B00000010;                     // Enable output on OC1A
				#endif
  
				TIMSK1 = B00000010;                     // Interrupt on compare match with OCR1A               
				TCCR1A = B01000011;                     // Fast PWM mode, will generate ISR() when it reaches OCR1A (pulse[0]), thus starting the PPM signal
				TCCR1B = B00011010;                     // Fast PWM, 8 prescaler (bit 2, disabled until PPM on), 16bits holds up to 65535, 8 PS puts our counter into 1/2 useconds (16MHz / 8 = 2MHz)        
				sei(); // Re-enable interrupts
    
			}

		} 

	} else {

		if(signalInfo.firstSignalEstablished) {

			if((currentTime - signalInfo.lastMessageSentTime) > configInfo.pingInterval) { // Signal is bad, send a ping to try restore connection

				sendPing(); 

			}
	
		}
    
	}

}

/* storePulse(index, us, inRangeLow, inRangeHigh) - map input values to PPM durations (min/max) and store pulse */

void storePulse(byte index, int inValue, int inRangeLow, int inRangeHigh) {

	int mappedPulse = map(inValue, inRangeLow, inRangeHigh, configInfo.ppmMinPulse, configInfo.ppmMaxPulse); // Map input value to pulse width
        if(TCNT1 > configInfo.ppmHighPulse) {  // Avoid PPM inversion by skipping this set, this will cause a max delay of 20ms in a servo position setting
  
          cli(); // Disable interrupts while this is being set
          ppmInfo.pulses[(index * 2) + 1] = mappedPulse; // Store new pulse width
          // DEBUG
          //pulses[(index * 2) + 1] = 2000;
          sei(); // Re-enable interrupts
          
        }

}


/* handleButtonUpdate() - Handle updates to the controls */

void handleButtonUpdate() {
  
	byte x;

	if(buttons[4] > 0) { // Handle navlight button
    
		lights[NAV_LIGHT].interval = configInfo.navlightInterval;  // enable navlight if button 5 is on
    
	} else {
    
		lights[NAV_LIGHT].interval = -1; // otherwise disable it
    
	}
  	for(x = 0 ; x < configInfo.buttonCount ; x++) {
 	 		  	
  		if(buttons[x] > 0) {
  		
  			if(configInfo.buttonPinMap[x] > 0 && configInfo.buttonPinMap[x] < 128) {
  			
  				digitalWrite(configInfo.buttonPinMap[x], HIGH);
  			
  			}
  		
  		} else {
  		
  			if(configInfo.buttonPinMap[x] > 0 && configInfo.buttonPinMap[x] < 128) {
  			
  				digitalWrite(configInfo.buttonPinMap[x], LOW);
  			
  			}
  		
  		}
  	
  	}
	#if DEBUG_LEVEL == 5
	for(x = 0 ; x < configInfo.ppmPulses ; x++) {

		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "---Pulse[%d]: %d-", x, pulses[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                              // Write debug message
    
	}
	#endif
  
}

/* Write a message back to the XBee port */

void writeXBeeMessage(messageState *msg, unsigned char msgType) {
 
	msg->messageBuffer[0] = MTYPE_BEGIN;                                                         // Message construction 
	msg->messageBuffer[1] = msgType;                                                             // Specify the message type
	msg->messageBuffer[2] = msg->length;                                                         // Message size
	msg->messageBuffer[msg->length - 1] = generateChecksum(msg->messageBuffer, msg->length - 1); // Fill in our checksum for the whole message
   
	Serial.write(msg->messageBuffer, msg->length);  // Write out the message
	signalInfo.lastMessageSentTime = millis();
     
}

/* Write a message back to the PPZ port */

#ifdef __AVR_ATmega644P__
void writePPZMessage(messageState *msg) {
   
	byte x;
	
	msg->messageBuffer[msg->length - 1] = '\0'; // End-of-string for last character replaces checksum

	for(x = 0 ; x < msg->length - PPZ_MSG_HEADER_SIZE; x++) {
	
		msg->messageBuffer[x] = msg->messageBuffer[x + PPZ_MSG_HEADER_SIZE]; // Shift everything PPZ_MSG_HEADER_SIZE to the left to drop the header
	
	}

	Serial1.write(msg->messageBuffer, msg->length - PPZ_MSG_HEADER_SIZE);  // Write out the message, minus the header size
     
}
#endif

/* Init our timer */

void initTimer() {
  
	cli();   
	#ifdef __AVR_ATmega644P__
	DDRD  &= B11011111; // Disable output on OC1A      
	#else
	DDRB  &= B11111101; // Disable output on OC1A
	#endif
	TIMSK1 = B00000000; // Disable interrupt on compare match
	TCCR1A = B00000000; // Disable fast PWM
	TCCR1B = B00000000; // Disable fast PWM, clock, and prescaler
	sei();
  
}

/* ISR - TIMER1_COMPAT_Vect, generates the PPM signal */

ISR(TIMER1_COMPA_vect) {

	OCR1A = ppmInfo.pulses[ppmInfo.currentPulse];     // Set OCR1A compare register to our next pulse
	ppmInfo.currentPulse++;                            // Increment the pulse counter
	if(ppmInfo.currentPulse >= configInfo.ppmPulses) { // If the pulse counter is too high reset it
    
		ppmInfo.currentPulse = 0;
    
	}
 
}
