
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

#include "joystickRC_structs.h"

/* This is the defining moment of the file */

#define DEBUG_LEVEL 0   // 1 - Messaging debugging
                        // 2 - Servo / pin output
                        // 3 - Signal continuity debugging (light 4 stays on if signal is ever lost)
                        // 4 - Signal continuity (with serial output)
                        // 5 - PPM registers
                        // 6 - PPM pulse values
                        // 7 - Only start debug message
                        // 8 - Report bad checksums

#ifdef __AVR_ATmega644P__
#define DEBUG_PIN1     4 // Pin for debug signaling
#define STATUS_LED_PIN 0 // Status LED pin
#define NAVLIGHT_PIN  18 // Navigation light pin  
#else
#define DEBUG_PIN1     12 // Pin for debug signaling   
#define STATUS_LED_PIN 13 // Status LED pin
#define NAVLIGHT_PIN   11 // Navigation light pin  
#endif

#define NAV_LIGHT    1
#define STATUS_LIGHT 0
#define FLASHING_LIGHTS 2

#define VERSION_MAJOR 3     // Major version #
#define VERSION_MINOR 2     // Minor #
#define VERSION_MOD   0     // Mod #
#define VERSION_TAG   "DBG" // Tag

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

#define MTYPE_PING_SZ            4    // Message types: Ping
#define MTYPE_PING_REPLY_SZ      4    // Ping reply
#define MTYPE_HEARTBEAT_SZ       3    // Heartbeat (no control update)
#define MTYPE_FULL_UPDATE_SZ    22    // Full update
#define MTYPE_SINGLE_SERVO_SZ    5    // Single servo
#define MTYPE_ALL_SERVOS_SZ     19    // Update all servos
#define MTYPE_BUTTON_UPDATE_SZ   6    // Buttons update

#define MSG_BUFFER_SIZE       256
#define LOST_MSG_THRESHOLD 1000UL    // How long without legit msg before handShook gets unset
#define HEARTBEAT_INTERVAL  500UL    // 500ms
#define PING_INTERVAL       100UL    // 100ms
#define PPZ_MSG_HEADER_SIZE     3    // PPZ msg header size in bytes

#define SERVO_COUNT   8     // # of servos
#define BUTTON_COUNT 12     // # of buttons on controller

#define PPM_MIN_PULSE 2000  // Min pulse length (1ms)
#define PPM_MAX_PULSE 4000  // Max pulse length (2ms)
#define PPM_HIGH_PULSE 400  // Delay between pulses (200us)
#define PPM_FREQUENCY 40000 // Frequency of PPM frame (20ms)
#define PPM_PULSES ((SERVO_COUNT * 2) + 2)  // How many pulses are there in the whole PPM (One 220us HIGH per servo, then 1ms-2ms LOW for servo pos, then 220us HIGH for pulse, then PPM_SYNC_PULSE LOW)
#define PPM_SYNC_PULSE (PPM_FREQUENCY - (SERVO_COUNT * (((PPM_MAX_PULSE + PPM_MIN_PULSE) / 2) + PPM_HIGH_PULSE))) // Duration of sync pulse

#define STATUS_INTERVAL_SIGNAL_LOST 100 // Toggle every 100ms
#define STATUS_INTERVAL_OK 1000         // Toggle every 1s

#define NAV_LIGHT_INTERVAL 1000          // Toggle every 1s

#define ACK_MIN_SIZE 10
#define ACK_MAX_SIZE 30

/* Various varibles to hold state info */

unsigned int servos[SERVO_COUNT];        // store servo states
unsigned int buttons[BUTTON_COUNT];      // store button states

boolean handShook = false;
unsigned char pingData;
unsigned long lastMessageTime = -1UL * LOST_MSG_THRESHOLD; // Time of last legit message, -100 initially so the PPM won't turn on until we get a real message
unsigned long lastMessageSentTime = 0UL;

ledBlinker lights[FLASHING_LIGHTS];  // blinking lights state

byte currentPulse = 0;        // The pulse being sent
boolean ppmON = false;
int pulses[PPM_PULSES];        // PPM pulses

messageState xbeeMsg;  // Message struct for messages from XBee line
#ifdef __AVR_ATmega644P__
messageState ppzMsg;  // Message struct for messages from PPZ line
#endif
#if DEBUG_LEVEL > 0
messageState dbgMsg;  // Message struct for outgoing debug messages
#endif

#ifdef __AVR_ATmega644P__
byte buttonPinMap[BUTTON_COUNT] = {1, 2, 3, 4, 5, -1, -1, 6, 7, 12, 14};
#else
byte buttonPinMap[BUTTON_COUNT] = {2, 3, 4, 5, -1, -1, -1, 7, 8, 10, A0};
#endif

/* Setup function */

void setup() {

	randomSeed(analogRead(0));          // Seed our random number gen with an unconnected pins static
	initControlState();                 // Initialise control state
	initOutputs();                      // Initialise outputs
	initPPM();                          // Set default PPM pulses
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
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----joystick2ppm version %d.%d.%d-%s... Open for debugging mode...-", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD, VERSION_TAG);  // Write a debug message leading and trailing dashes will be replaced with header and checksums
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);  // Send debug message
	#endif
  
	#if DEBUG_LEVEL == 3 || DEBUG_LEVEL == 4
	pinMode(DEBUG_PIN1, OUTPUT);  //  DEBUG - Pin will light permanently if signal is lost
	#endif

}

/* Loop function */

void loop() {

	int x;
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

/* initControlState() - Zeroes out everythang */

void initControlState() {

	int x;

  	// Zero out all buttons and servos

	for(x = 0 ; x < SERVO_COUNT ; x++) {

		servos[x] = 0;

	}

	for(x = 0 ; x < BUTTON_COUNT ; x++) {

		buttons[x] = 0;

	}

}

/* initPPM - What do you think? */

void initPPM() {

	int x;
	int midPPMPulse = (PPM_MIN_PULSE + PPM_MAX_PULSE) / 2;  

	for (x = 0 ; x < (SERVO_COUNT + 1) ; x++) {
    
		pulses[x * 2] = PPM_HIGH_PULSE;  // DEBUG
		pulses[(x * 2) + 1] = midPPMPulse; // Set all PPM pulses to halfpulse
    
	}
	pulses[PPM_PULSES - 1] = PPM_SYNC_PULSE; // Sync pulse is before 0 length pulse

	#if DEBUG_LEVEL == 5
	for(x = 0 ; x < PPM_PULSES ; x++) {
   
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Pulse[%d]: %d -", x, pulses[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                              // Write debug message
    
	}
	#endif
	currentPulse = 0; // init currentPulse

}

/* initOutputs() - Set output pins up */

void initOutputs() {

  	int x;
  	
  	for(x = 0 ; x < BUTTON_COUNT ; x++) {
  	
  		if(buttonPinMap[x] > 0) {
  		
  			pinMode(buttonPinMap[x], OUTPUT);
  		
  		}
  	
  	}
  
        lights[NAV_LIGHT].pin = NAVLIGHT_PIN;
        lights[NAV_LIGHT].state = false;
        lights[NAV_LIGHT].lastChanged = 0;
        lights[NAV_LIGHT].interval = -1;
        
        lights[STATUS_LIGHT].pin = STATUS_LED_PIN;
        lights[STATUS_LIGHT].state = false;
        lights[STATUS_LIGHT].lastChanged = 0;
        lights[STATUS_LIGHT].interval = STATUS_INTERVAL_SIGNAL_LOST;
        
        for(x = 0 ; x < FLASHING_LIGHTS ; x++) {
  
	    pinMode(lights[x].pin, OUTPUT); // Mark pin as output

        }
  
}

/* initMessage() - Initialise message */

boolean initMessage(messageState *msg) {

        int x;
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
	
	if(msg->length < 0) { // Message has < 0 length, check if anything in messageBuffer can fill that in

		msg->length = getMessageLength(msg);

	}

	if(msg->readBytes < msg->length || msg->length < 0) {  // We either aren't done reading the message or we don't have MSG_BEGIN and/or MTYPE and/or PARAM to tell us the real length

		if(Serial.available() > 0)  {  // Byte is availabe, read it

			testByte = Serial.read();

			if(msg->readBytes == 0) { // Haven't got MSG_BEGIN yet, look for it

				if(testByte == MSG_BEGIN) { // Beginning of a messge

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

	} else { // Message is finished, process it

		int x, y;	

		if(testChecksum(msg->messageBuffer, msg->length)) { // Checksum passed, process message..  

			processMessage(msg);
			if(msg->messageBuffer[1] != MTYPE_PING) {

				lastMessageTime = millis(); // Set last message time, except for from a ping

			}

		} else {

			x = 1;
			msg->length = -1;

			while(x < msg->readBytes && msg->length < 0) { // Try to find out if we have a legit message later in the buffer

				if(msg->messageBuffer[x] == MSG_BEGIN) { // Looks like a MSG_BEGIN

					for(y = 0 ; y < x ; y++) {

						msg->messageBuffer[y] = msg->messageBuffer[y+x]; // Push everything to the left x spaces

					}

					msg->readBytes = msg->length - x;    // adjust number of read bytes 
					msg->length = getMessageLength(msg); // get the new length, if it's an invalid message we'll get -1
					x = 0;

				}

				x++;

			}

		}

		if(msg->readBytes > msg->length) { // We've got more bytes in the buffer than the message length, shift everything to the left msg->length

			for(x = 0 ; x < (msg->readBytes - msg->length) ; x++) {

				msg->messageBuffer[x] = msg->messageBuffer[x + msg->length];

			}
			msg->readBytes = msg->readBytes - msg->length;
			msg->length = getMessageLength(msg);

		} else {

			// Clear out message so it's ready to be used again	
			for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

				msg->messageBuffer[x] = '\0';

			}
			msg->readBytes = 0;   // Zero out readBytes
			msg->length = -1;     // Set message length to -1		
			
		
		}
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
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----PPZBYTE[%d - CSLA: %d]: %x-", dbgMsg.readBytes, commandsSinceLastAck, testByte);
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);
		#endif		

		msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
		msg->readBytes++;			       // Increment readBytes
                msg->length++;                                 // Increment length
		
		if(testByte == '\n') { // This is the message end, relay the message to GCS and reset msg
		
			writeXBeeMessage(msg, MTYPE_PPZ);

			msg->readBytes = PPZ_MSG_HEADER_SIZE;  // Leave room for header to be added
                        msg->length = PPZ_MSG_HEADER_SIZE;
			int x;	

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

	if(msg->readBytes > 1) { // Do zero-parameter types first, if we can't find one, see if we have enough characters for one of the parametered types

		if(msg->messageBuffer[1] == MTYPE_HEARTBEAT) { // Do 3-char long messages first

			return MTYPE_HEARTBEAT_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_PING || msg->messageBuffer[1] == MTYPE_PING_REPLY) { // 4 char messages

			return MTYPE_PING_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_SINGLE_SERVO || msg->messageBuffer[1] == MTYPE_BUTTON_UPDATE) {

			return MTYPE_SINGLE_SERVO_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_ALL_SERVOS) {
		
			return MTYPE_ALL_SERVOS_SZ;

		} else if(msg->messageBuffer[1] == MTYPE_FULL_UPDATE) {
	
			return MTYPE_FULL_UPDATE_SZ;

		} else if(msg->readBytes > 2) {  // Didn't find any non-parameter message types, let's see if we have a parametered one

			if(msg->messageBuffer[1] == MTYPE_PPZ || msg->messageBuffer[1] == MTYPE_DEBUG) {

				return (int)msg->messageBuffer[2];  // PPZ & Debug messages have length as param

			} else if(msg->messageBuffer[1] == MTYPE_VAR_SERVOS) {
		
				return 4 + ((int)msg->messageBuffer[2] * 2);  // Convert servo count to # of bytes (2 bytes per servo + begin + type + param + check)

			} else {

				return -1; // No valid message types to provide length found

			}

		} 

	} else {

		return -1; // Haven't read enough bytes yet

	}

}

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(messageState *msg) {

        int x;
	unsigned char msgType = msg->messageBuffer[1];

	if(msgType == MTYPE_PING) { // We got a ping, send an ack

		sendAck(msg);

	} else if(msgType == MTYPE_PING_REPLY) {  // Handle the message, since it got past checksum it has to be legit

		if(msg->messageBuffer[2] == pingData) { //  See if the payload matches the ping packet we sent out
		
			handShook = true;

		}

	} else if(msgType == MTYPE_SINGLE_SERVO) {

		int servoNum = 0;
		int servoPos = 0;

		servoNum = (msg->messageBuffer[2] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
		servoPos = (msg->messageBuffer[2] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
		servoPos = servoPos | msg->messageBuffer[3]; // Store last 8 bits of servo position
		storePulse(servoNum, servoPos, 0, 180);
		servos[servoNum] = servoPos;

	} else if(msgType == MTYPE_VAR_SERVOS) {

		int servoCount = 0;
		int servoNum;
		int servoPos;

		servoCount = msg->messageBuffer[2];

		for(x = 0 ; x < servoCount ; x++) {

			servoNum = (msg->messageBuffer[(x * 2) + 3] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
			servoPos = (msg->messageBuffer[(x * 2) + 3] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
			servoPos = servoPos | msg->messageBuffer[(x * 2) + 4]; // Store last 8 bits of servo position
			storePulse(servoNum, servoPos, 0, 180);
			servos[servoNum] = servoPos;

		}

	} else if(msgType == MTYPE_ALL_SERVOS) {

		int servoNum;
		int servoPos;

		for(x = 0 ; x < SERVO_COUNT ; x++) {

			servoNum = (msg->messageBuffer[(x * 2) + 2] >> 2) & B00111111; // Binary: 0011 1111, strip out any added 1s
			servoPos = (msg->messageBuffer[(x * 2) + 2] & B00000011) << 8; // Binary: 0000 0011, strip out servo number, shift top 2 bits of servo pos over
			servoPos = servoPos | msg->messageBuffer[(x * 2) + 3]; // Store last 8 bits of servo position
			storePulse(servoNum, servoPos, 0, 180);
			servos[servoNum] = servoPos;

		}

	} else if(msgType == MTYPE_BUTTON_UPDATE) {

		for(x = 0 ; x < 3 ; x++) {  // This loop handles 4 buttons at once since each uses 2 bits and we read in 1 byte (2 bits * 4 = 8 bits = 1 byte)

			buttons[(x * 4)] = (msg->messageBuffer[x + 2] & B11000000) >> 6;     // Bitwise and against our byte to strip away other button values, then bitshift to 0th and 1st positions
			buttons[(x * 4) + 1] = (msg->messageBuffer[x + 2] & B00110000) >> 4; // Same, you can see the bitmask shift to the right as we work out way down the byte
			buttons[(x * 4) + 2] = (msg->messageBuffer[x + 2] & B00001100) >> 2; // Same
			buttons[(x * 4) + 3] = (msg->messageBuffer[x + 2] & B00000011);      // No bitshift here since our bits are already in 0th and 1st pos.

	        }

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
	int x;

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x]; // Generate checksum

	}

	return checksum;

}

/* testChecksum(message, length) - Test if the last byte checksum is good */

int testChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 1
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----CHKMSG:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message
	#endif

	for(x = 0 ; x < length ; x++) {

		#if DEBUG_LEVEL == 1
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----%x-", (unsigned int)message[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message
		#endif
                checksum = checksum ^ (unsigned int)message[x];  // Test this message against its checksum (last byte)

	}
	#if DEBUG_LEVEL == 1
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----CHK: %x-", checksum); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message
	#endif

	if(checksum == 0x00) {

		return true;  // Checksum passed!

	} else {

		#if DEBUG_LEVEL == 8
		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Bad checksum (length: %d): %2x-", length, checksum); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                         // Write debug message		
		#endif
		return false;

	}

}

/* sendHeartbeat() - Send heartbeat */

void sendHeartbeat() {

	unsigned char heartbeat[MTYPE_HEARTBEAT_SZ];

        #if DEBUG_LEVEL == 1
      	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Sending HEARTBEAT:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                   // Write debug message
	#endif 

	heartbeat[0] = MSG_BEGIN;
	heartbeat[1] = MTYPE_HEARTBEAT;
	heartbeat[2] = generateChecksum(heartbeat, MTYPE_HEARTBEAT_SZ - 1); // Store our checksum as the last byte
	
	Serial.write(heartbeat, MTYPE_HEARTBEAT_SZ);     // Send the sync ACK
	lastMessageSentTime = millis();

}

/* sendPing() - Send ping! */

void sendPing() {

	unsigned char ping[4];
	pingData = random(0, 256);

	ping[0] = MSG_BEGIN;
	ping[1] = MTYPE_PING;
	ping[2] = pingData;
	ping[3] = generateChecksum(ping, MTYPE_PING_SZ - 1); // Store our checksum as the last byte

	Serial.write(ping, MTYPE_PING_SZ);     // Send the ping
	lastMessageSentTime = millis();

}

/* sendAck(message) - Send ack! */

void sendAck(messageState *msg) {

	unsigned char pingReply[4];

        #if DEBUG_LEVEL == 1
      	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Sending PING ACK:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                   // Write debug message
	#endif 

	pingReply[0] = MSG_BEGIN;
	pingReply[1] = MTYPE_PING_REPLY;
	pingReply[2] = msg->messageBuffer[2];
	pingReply[3] = generateChecksum(pingReply, MTYPE_PING_REPLY_SZ - 1); // Store our checksum as the last byte

	Serial.write(pingReply, MTYPE_PING_REPLY_SZ);     // Send the sync ACK
	lastMessageSentTime = millis();

}

/* updateLights() - Update lights based on things */

void updateLights() {

	int x;
        
	for(x = 0 ; x < FLASHING_LIGHTS ; x++) {
          
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

/* handleSignal() - Check the signal state and make necessary updates */

void handleSignal() {

	unsigned long currentTime = millis(); // get current time
	if(handShook) { // Signal is still good last we checked

		if((currentTime - lastMessageTime) > LOST_MSG_THRESHOLD) { // Check if the signal is actually still good
     
			cli(); // Do not allow timer ppm disabling to be interrupted
			handShook = false;                               // If we haven't received a message in > LOST_MSG_THRESHOLD set handShook = false
			ppmON = false;                                   // Disable PPM
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
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Stopping PPM, lostSignal = true-"); // Build debug message
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                               // Write debug message
			#endif
			#if DEBUG_LEVEL == 4
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----commandsSinceLastAck: %d currentTime: %lu lastMessageTime: %lu diff: %lu > %lu-", commandsSinceLastAck, currentTime, lastMessageTime, (currentTime - lastMessageTime), LOST_MSG_THRESHOLD); // Build debug message
			writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                               // Write debug message
			#endif
			lights[STATUS_LIGHT].interval = STATUS_INTERVAL_SIGNAL_LOST; // Set status LED interval to signal lost
			#if DEBUG_LEVEL == 3 || DEBUG_LEVEL == 4
			digitalWrite(DEBUG_PIN1, HIGH);
			#endif
			sei(); // Re-enable interrupts

		} else { // The signal is good, do we need to send a heartbeat?

			if((currentTime - lastMessageSentTime) > HEARTBEAT_INTERVAL) {

				sendHeartbeat();

			}

			if(!ppmON) {  // Restart PPM since it was off

				cli();  // This shouldn't get interrupted since PPM is off but just to be safe..

				ppmON = true;                           // turn on PPM status flag
				lights[STATUS_LIGHT].interval = STATUS_INTERVAL_OK; // Set our status LED interval to OK

				#if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 4
				dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Starting PPM, lostSignal = false-"); // Build debug message
				writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                               // Write debug message
				#endif        
		        
				TCCR1B = B00001000;                     // CTC mode, clock disabled, OCR1A will never be reached by TCNT1 'coz no clock is running
				TCCR1A = B11000000;                     // CTC, set OC1A HIGH on match
	
				OCR1A = 0xFFFF;                         // Make OCR1A max so it doesn't get hit
	
				TCCR1C = B10000000;                     // Force match, should set pin high, WILL NOT generate ISR() call        

				OCR1A = pulses[0];                      // Set OCR1A to pulse[0], this won't actually matter until we set TCCR1A and TCCR1B at the end to enable fast PWM
				currentPulse = 1;                       // Set currentPulse to 1 since there will be no ISR() call to increment it

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

		if((currentTime - lastMessageSentTime) > PING_INTERVAL) { // Signal is bad, send a ping to try restore connection

			sendPing(); 

		}
    
	}

}

/* storePulse(index, us, inRangeLow, inRangeHigh) - map input values to PPM durations (min/max) and store pulse */

void storePulse(byte index, int inValue, int inRangeLow, int inRangeHigh) {

	int mappedPulse = map(inValue, inRangeLow, inRangeHigh, PPM_MIN_PULSE, PPM_MAX_PULSE); // Map input value to pulse width
        if(TCNT1 > PPM_HIGH_PULSE) {  // Avoid PPM inversion by skipping this set, this will cause a max delay of 20ms in a servo position setting
  
          cli(); // Disable interrupts while this is being set
          pulses[(index * 2) + 1] = mappedPulse; // Store new pulse width
          // DEBUG
          //pulses[(index * 2) + 1] = 2000;
          sei(); // Re-enable interrupts
          
        }

}


/* handleButtonUpdate() - Handle updates to the controls */

void handleButtonUpdate() {
  
	int x;

	if(buttons[4] > 0) { // Handle navlight button
    
		lights[NAV_LIGHT].interval = NAV_LIGHT_INTERVAL;  // enable navlight if button 5 is on
    
	} else {
    
		lights[NAV_LIGHT].interval = -1; // otherwise disable it
    
	}
  	for(x = 0 ; x < BUTTON_COUNT ; x++) {
  	 		  	
  		if(buttons[x] > 0) {
  		
  			if(buttonPinMap[x] > 0) {
  			
  				digitalWrite(buttonPinMap[x], HIGH);
  			
  			}
  		
  		} else {
  		
  			if(buttonPinMap[x] > 0) {
  			
  				digitalWrite(buttonPinMap[x], LOW);
  			
  			}
  		
  		}
  	
  	}
	#if DEBUG_LEVEL == 5
	for(x = 0 ; x < PPM_PULSES ; x++) {

		dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Pulse[%d]: %d-", x, pulses[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MTYPE_DEBUG);                                              // Write debug message
    
	}
	#endif
  
}

/* Write a message back to the XBee port */

void writeXBeeMessage(messageState *msg, unsigned char msgType) {
 
	msg->messageBuffer[0] = MSG_BEGIN;                                                         // Message construction 
	msg->messageBuffer[1] = msgType;                                                           // Specify the message type
	msg->messageBuffer[2] = msg->length;                                                       // Message size
	msg->messageBuffer[msg->length - 1] = generateChecksum(msg->messageBuffer, msg->length - 1); // Fill in our checksum for the whole message
   
	Serial.write(msg->messageBuffer, msg->length);  // Write out the message
	lastMessageSentTime = millis();
     
}

/* Write a message back to the PPZ port */

#ifdef __AVR_ATmega644P__
void writePPZMessage(messageState *msg) {
   
	int x;
	
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

	OCR1A = pulses[currentPulse];    // Set OCR1A compare register to our next pulse
	currentPulse++;                  // Increment the pulse counter
	if(currentPulse >= PPM_PULSES) { // If the pulse counter is too high reset it
    
		currentPulse = 0;
    
	}
 
}
