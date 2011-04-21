
/* joystick2ppm - Paul Mandal (paul.mandal@gmail.com)
 * 2.0 - Recieves encoded servo messages from joystick app on GCS
 *     - Updates PPM pulses at 100Hz
 *     - Produces PPM frame every 20ms
 *     - Relays non-servo USART messages to second USART
 *     - Relays second USART (PPZ) messages to USART / GCS
 *     - Uses timers and cool stuff like that
 * 
 * Thanks to the Arduino community, everyone on the Sanguino team, and everyone involved in science and maths and them things.
 * And fanks to my friend Liz for showing me that science ain't just for nerdy blokes with pocket protectas and spectacles, but can also be quite a laugh.
 *
 * For reference this is coded for the ATmega644P, it has these pins: 
 * PWM: 3, 4, 12, 13, 14, 15
 * Digital I/O: 0, 1, 2, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 23
 * USART: 8, 9, 10, 11
 * ADC: 24, 25, 26, 27, 28, 29 (I think?)
 */

/* Including things */

#include "joystickRC_structs.h"

/* This is the defining moment of the file */

#define DEBUG_LEVEL 4   // 1 - Messaging debugging
                        // 2 - Servo / pin output
                        // 3 - Signal continuity debugging (light 4 stays on if signal is ever lost)
                        // 4 - Signal continuity (with serial output)
                        // 5 - PPM registers
                        // 6 - PPM pulse values
#define DEBUG_PIN1  4   // Pin for debug signaling   
                     

#define VERSION_MAJOR 2     // Major version #
#define VERSION_MINOR 9     // Minor #
#define VERSION_MOD   5     // Mod #

#define MSG_BEGIN     0xFF                    // Begin of control message indicator byte
#define MSG_TYPE_CTRL 0x01                    // Control update message type indicator
#define MSG_TYPE_CFG  0x02		      // Configuration update
#define MSG_TYPE_PPZ  0x03                    // Message to/from PPZ
#define MSG_TYPE_DBG  0x04                    // Debug message
#define MSG_TYPE_SYNC 0xFE                    // Sync message type indicator
#define MSG_BUFFER_SIZE 256 		      // Message buffer size in bytes
#define MSG_HEADER_SIZE 4     		      // Message header size in bytes
#define CMDS_PER_ACK  50                      // Client will assume lost signal if we send this many commands without an ack
#define LOST_MSG_THRESHOLD 60UL               // How long without legit msg before lostSignal gets set

#define SERVO_COUNT 8       // # of servos
#define BUTTON_COUNT 12     // # of buttons on controller

#define PPM_MIN_PULSE 1000  // Min pulse length (1ms)
#define PPM_MAX_PULSE 2000  // Max pulse length (2ms)
#define PPM_HIGH_PULSE 200  // Delay between pulses (200us)
#define PPM_FREQUENCY 20000 // Frequency of PPM frame (20ms)
#define PPM_PULSES ((SERVO_COUNT * 2) + 2)  // How many pulses are there in the whole PPM (One 220us HIGH per servo, then 1ms-2ms LOW for servo pos, then 220us HIGH for pulse, then PPM_SYNC_PULSE LOW)
#define PPM_SYNC_PULSE (PPM_FREQUENCY - (SERVO_COUNT * (((PPM_MAX_PULSE + PPM_MIN_PULSE) / 2) + PPM_HIGH_PULSE))) // Duration of sync pulse

#define STATUS_LED_PIN 0
#define STATUS_INTERVAL_SIGNAL_LOST 100 // Toggle every 100ms
#define STATUS_INTERVAL_OK 1000         // Toggle every 1s

#define NAVLIGHT_PIN 18                 // Navigation light pin
#define NAVLIGHT_INTERVAL 1000          // Toggle every 1s

#define ACK_MIN_SIZE 10
#define ACK_MAX_SIZE 30

/* Various varibles to hold state info */

unsigned int servos[SERVO_COUNT];        // store servo states
unsigned int buttons[BUTTON_COUNT];      // store button states

boolean lostSignal = true;        // lostSignal state
unsigned long lastMsgTime = -1UL * LOST_MSG_THRESHOLD; // Time of last legit message, -100 initially so the PPM won't turn on until we get a real message

unsigned int navlightInterval = NAVLIGHT_INTERVAL; // Interval for navigation lights
unsigned long navlightLastTime = 0;      // Navigation light last 
boolean navlightState = false;           // Navigation light LED state
boolean navlightEnabled = false;         // Enable/disable navigation lights

unsigned long lastStatusLEDTime = 0;     // Time of last status LED change
unsigned int statusLEDInterval = STATUS_INTERVAL_SIGNAL_LOST; // Current status LED toggle interval
boolean statusLEDState = false;          // Status LED state

byte currentPulse = 0;        // The pulse being sent
boolean ppmON = false;
int pulses[PPM_PULSES];        // PPM pulses

messageState xbeeMsg;  // Message struct for messages from XBee line
messageState ppzMsg;  // Message struct for messages from PPZ line
#if DEBUG_LEVEL > 0
messageState dbgMsg;  // Message struct for outgoing debug messages
#endif

int commandsSinceLastAck = 0;
unsigned char ackMsg[ACK_MAX_SIZE];

/* Arduino is racist against function prototypes 

void initControlState();
void initPPM();
void initOutputs();
boolean initMessage(messageState *message);
void initTimer();
boolean checkXBeeMessages(messageState *msg);
void processMessage(unsigned char *message, int length);
unsigned char generateChecksum(unsigned char *message, int length);
boolean testChecksum(unsigned char *message, int length);
void sendAck();
void updateStatusLED();
void updateNavigationLights();
void checkSignal();
void storePulse(byte index, int inValue, int inRangeLow, int inRangeHigh);
void handleControlUpdate();*/

/* Setup function */

void setup() {

	randomSeed(analogRead(0));          // Seed our random number gen with an unconnected pins static
	initControlState();                 // Initialise control state
	initOutputs();                      // Initialise outputs
	initPPM();                          // Set default PPM pulses
	initMessage(&xbeeMsg);              // Init our XBee message
	initMessage(&ppzMsg);               // Init our PPZ message
	ppzMsg.readBytes = MSG_HEADER_SIZE; // Leave room for header addition to PPZ message
	initTimer();                        // Init our timer
	Serial.begin(115200);               // Open XBee/GCS Serial
	Serial.flush();
	Serial1.begin(115200);              // Open PPZ Serial
	Serial1.flush();
  
	#if DEBUG_LEVEL > 0
	initMessage(&dbgMsg);      // Init our debug message
	dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----joystick2ppm version %d.%d.%d... Open for debugging mode...-", VERSION_MAJOR, VERSION_MINOR, VERSION_MOD);  // Write a debug message leading and trailing dashes will be replaced with header and checksums
	writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);  // Send debug message
	#endif
  
	#if DEBUG_LEVEL == 3 || DEBUG_LEVEL == 4
	pinMode(DEBUG_PIN1, OUTPUT);  //  DEBUG - Pin will light permanently if signal is lost
	#endif

}

/* Loop function */

void loop() {

	int x;
	updateStatusLED();        // Check if we need to toggle the status LED
	updateNavigationLights(); // Update Navigation lights

	for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) { // checkMessage functions should be run with a much higher frequency than the LED updates or checkSignal()
  
		checkXBeeMessages(&xbeeMsg);        // Check for incoming XBee messages
		checkPPZMessages(&ppzMsg);		// Check for incoming PPZ messages

	}

	checkSignal();            // Check if the signal is still good

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
   
		dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Pulse[%d]: %d -", x, pulses[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                              // Write debug message
    
	}
	#endif
	currentPulse = 0; // init currentPulse

}

/* initOutputs() - Set output pins up */

void initOutputs() {
  
	pinMode(STATUS_LED_PIN, OUTPUT); // Status LED Pin
	pinMode(NAVLIGHT_PIN, OUTPUT);   // Navlight LED(s) Pin
  
}

/* initMessage() - Initialise message */

boolean initMessage(messageState *msg) {

        int x;
	msg->readBytes = 0;
	msg->length = MSG_HEADER_SIZE; // Init message.length as header length size
	if((msg->messageBuffer = (unsigned char*)calloc(MSG_BUFFER_SIZE, sizeof(char))) != NULL) {
	
                for(x = 0 ; x < MSG_HEADER_SIZE ; x++) {
                  
                  msg->messageBuffer[x] = '\0';
                  
                }
		return true; // calloc() worked
	
	} else {
	
		return false; // calloc() failed
	
	} 
	
}

/* checkXBeeMessages() - Check for and handle any incoming messages */

boolean checkXBeeMessages(messageState *msg) {

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

		if(Serial.available() > 0) {

			testByte = Serial.read();  // Read our byte		

			#if DEBUG_LEVEL == 1
			dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----BYTE[%d/%d - CSLA: %d]: %x-", msg->readBytes, msg->length, commandsSinceLastAck, testByte); // Build debug message
			writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                              // Write debug message
			#endif		

			msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
			msg->readBytes++;			  // Increment readBytes
	
			return true;
	
		} else {

			return false;
	
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

boolean checkPPZMessages(messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(Serial1.available() > 0) {

		testByte = Serial1.read();  // Read our byte		

		#if DEBUG_LEVEL == 1
		dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----PPZBYTE[%d - CSLA: %d]: %x-", dbgMsg.readBytes, commandsSinceLastAck, testByte);
		writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);
		#endif		

		msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
		msg->readBytes++;			       // Increment readBytes
		
		if(testByte == '\0') { // This is the message end, relay the message to GCS and reset dbgMsg
		
			writeXBeeMessage(msg, MSG_TYPE_PPZ);
			msg->readBytes = MSG_HEADER_SIZE;  // Leave room for header to be added
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

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(messageState *msg) {

        int x;
	unsigned char msgType = msg->messageBuffer[1];

	#if DEBUG_LEVEL == 1
	dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----CSLA: %d-", commandsSinceLastAck); // Build debug message
	writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                                // Write debug message
	#endif

        commandsSinceLastAck++;

	if(msgType == MSG_TYPE_SYNC) {  // Handle the message, since it got past checksum it has to be legit

		 sendAck(); // Send ACK for the SYNC signal

	} else if(msgType == MSG_TYPE_CTRL) { // Handle updating the controls

 		lastMsgTime = millis(); // Only command messages count for this
        	lostSignal = false;     // Message was legit, update lostSignal and lastMsgTime
		/* MSG structure - [BEGIN_MSG] [MSG_TYPE] [SERVOS] [BUTTONS] [CHECKSUM]
	         * BEGIN_MSG - 1 byte  - 1 byte msg marker
		 * MSG_TYPE  - 1 byte  - 1 byte msg type marker
                 * MSG_LEN   - 1 byte  - 1 byte msg length
                 * HDR_CHK   - 1 byte  - 1 byte msg header checksum
	         * SERVOS    - 8 bytes - 1 byte per servo
	         * BUTTONS   - 3 bytes - 2 bits per pin (allow more than on/off, e.g. 3-pos switch)
	         * CHECKSUM  - 1 byte  - 1 byte XOR checksum
	         */
          
	        for(x = 0 ; x < SERVO_COUNT ; x++) {

	          servos[x] = msg->messageBuffer[x + MSG_HEADER_SIZE]; // Set latest servo values from msg

	        }

	        for(x = 0 ; x < 3 ; x++) {  // This loop handles 4 buttons at once since each uses 2 bits and we read in 1 byte (2 bits * 4 = 8 bits = 1 byte)

	          buttons[(x * 4)] = (msg->messageBuffer[x + SERVO_COUNT + MSG_HEADER_SIZE] & B11000000) >> 6;     // Bitwise and against our byte to strip away other button values, then bitshift to 0th and 1st positions
	          buttons[(x * 4) + 1] = (msg->messageBuffer[x + SERVO_COUNT + MSG_HEADER_SIZE] & B00110000) >> 4; // Same, you can see the bitmask shift to the right as we work out way down the byte
		  buttons[(x * 4) + 2] = (msg->messageBuffer[x + SERVO_COUNT + MSG_HEADER_SIZE] & B00001100) >> 2; // Same
	          buttons[(x * 4) + 3] = (msg->messageBuffer[x + SERVO_COUNT + MSG_HEADER_SIZE] & B00000011);      // No bitshift here since our bits are already in 0th and 1st pos.

	        }

                handleControlUpdate();

	} else if(msgType == MSG_TYPE_PPZ) { // Handle PPZ message
	
		writePPZMessage(msg);
	
	} else if(msgType == MSG_TYPE_CFG) { // Handle configuration message
	}

        if(commandsSinceLastAck > CMDS_PER_ACK) {
          
           sendAck();  // Send an ACK since we've passed the CMDS_PER_ACK limit (note sendAck will zero out commandsSinceLastAck)
           
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
	dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----CHKMSG:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                         // Write debug message
	#endif

	for(x = 0 ; x < length ; x++) {

		#if DEBUG_LEVEL == 1
		dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----%x-", (unsigned int)message[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                         // Write debug message
		#endif
                checksum = checksum ^ (unsigned int)message[x];  // Test this message against its checksum (last byte)

	}
	#if DEBUG_LEVEL == 1
	dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----CHK: %x CSLA: %d-", checksum, commandsSinceLastAck); // Build debug message
	writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                         // Write debug message
	#endif

	if(checksum == 0x00) {

		return true;  // Checksum passed!

	} else {

		return false;

	}

}

/* sendAck() - Send SYNC acknowledgement message */

void sendAck() {

        unsigned int checksum;
	int x, msgSize;

        msgSize = random(ACK_MIN_SIZE, ACK_MAX_SIZE); // Sync message is between ACK_MIN_SIZE and ACK_MAX_SIZE chars
        
        #if DEBUG_LEVEL == 1
      	dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Sending SYNC ACK:-"); // Build debug message
	writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                   // Write debug message
	#endif 

	ackMsg[0] = MSG_BEGIN;  // Use our msg buffer to write back a sync reply
	ackMsg[1] = MSG_TYPE_SYNC;  // Sync reply will have same format (msgBegin, msgType, random chars, checksum)
        ackMsg[2] = msgSize; // Message length
        ackMsg[3] = generateChecksum(ackMsg, MSG_HEADER_SIZE - 1); // Header checksum
        for(x = 4 ; x < msgSize ; x++) { 

		ackMsg[x] = (unsigned char)random(0, 254); // Fill all but the last character with random bytes

	}
        ackMsg[msgSize - 1] = generateChecksum(ackMsg, msgSize - 1); // Store our message checksum
	Serial.write(ackMsg, msgSize);     // Send the sync ACK
        Serial.flush();                          // Flush the serial buffer since it may be full of garbage
        commandsSinceLastAck = 0;               // Set commandsSinceLastAck to 0

}

/* updateStatusLED() - Update status LED based on things */

void updateStatusLED() {

	unsigned long currentTime = millis(); // get current time
	if(currentTime - lastStatusLEDTime > statusLEDInterval) {

		lastStatusLEDTime = currentTime;              // If more time than statusLEDInterval has passed, replace lastStatusLEDTime with currentTime
		statusLEDState = !statusLEDState;             // Flip statusLEDState
		digitalWrite(STATUS_LED_PIN, statusLEDState); // Display status LED

	}

}

/* updateNavigationLights() - Update status LED based on things */

void updateNavigationLights() {

	if(navlightEnabled) {
    
		unsigned long currentTime = millis(); // get current time
		if(currentTime - navlightLastTime > navlightInterval) {

			navlightLastTime = currentTime;              // If more time than navlightInterval has passed, replace navlightLastTime with currentTime
			navlightState = !navlightState;              // Flip navlightState
			digitalWrite(NAVLIGHT_PIN, navlightState);   // Display navlight LED

		}
  
	} else {
    
		digitalWrite(NAVLIGHT_PIN, false);  // Navlights are turned off
    
	}

}

/* checkSignal() - Check the signal state and make necessary updates */

void checkSignal() {

	unsigned long currentTime = millis(); // get current time
	if((currentTime - lastMsgTime) > LOST_MSG_THRESHOLD) {

		if(!lostSignal) {
      
			cli(); // Do not allow timer ppm disabling to be interrupted
			lostSignal = true;                               // If we haven't received a message in > LOST_MSG_THRESHOLD set lostSignal
			ppmON = false;                                   // Disable PPM
			TIMSK1 = B00000000;                              // Disable interrupt on compare match
			TCCR1A = B00000000;                              // Disable fast PWM     
			TCCR1B = B00000000;                              // Disable fast PWM, clock, and prescaler
      
			TCCR1A = B10000000;                              // Set the pin to go low on compare match
			TCCR1C = B10000000;                              // Force match, this will force the pin low
			DDRD  &= B11011111;                              // Disable output on OC1A      
			#if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 4
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Stopping PPM, lostSignal = true-"); // Build debug message
			writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                               // Write debug message
			#endif
			#if DEBUG_LEVEL == 4
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----commandsSinceLastAck: %d currentTime: %lu lastMsgTime: %lu diff: %lu > %lu-", commandsSinceLastAck, currentTime, lastMsgTime, (currentTime - lastMsgTime), LOST_MSG_THRESHOLD); // Build debug message
			writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                               // Write debug message
			#endif
			statusLEDInterval = STATUS_INTERVAL_SIGNAL_LOST; // Set status LED interval to signal lost
			#if DEBUG_LEVEL == 3 || DEBUG_LEVEL == 4
			digitalWrite(DEBUG_PIN1, HIGH);
			#endif
			sei(); // Re-enable interrupts
      
		}

	} else {
    
		if(!ppmON) {  // Restart PPM since it was off

			cli();  // This shouldn't get interrupted since PPM is off but just to be safe..

			ppmON = true;                           // turn on PPM status flag
			statusLEDInterval = STATUS_INTERVAL_OK; // Set our status LED interval to OK

			#if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 4
			dbgMsg.length = snprintf((char *)dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Starting PPM, lostSignal = false-"); // Build debug message
			writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                               // Write debug message
			#endif        
        
			TCCR1B = B00001000;                     // CTC mode, clock disabled, OCR1A will never be reached by TCNT1 'coz no clock is running
			TCCR1A = B11000000;                     // CTC, set OC1A HIGH on match

			OCR1A = 0xFFFF;                         // Make OCR1A max so it doesn't get hit

			TCCR1C = B10000000;                     // Force match, should set pin high, WILL NOT generate ISR() call        

			OCR1A = pulses[0];                      // Set OCR1A to pulse[0], this won't actually matter until we set TCCR1A and TCCR1B at the end to enable fast PWM
			currentPulse = 1;                       // Set currentPulse to 1 since there will be no ISR() call to increment it

			DDRD  |= B00100000;                     // Enable output on OC1A
  
			TIMSK1 = B00000010;                     // Interrupt on compare match with OCR1A               
			TCCR1A = B01000011;                     // Fast PWM mode, will generate ISR() when it reaches OCR1A (pulse[0]), thus starting the PPM signal
			TCCR1B = B00011010;                     // Fast PWM, 8 prescaler (bit 2, disabled until PPM on), 16bits holds up to 65535, 8 PS puts our counter into useconds (16MHz / 8 * 2 = 1MHz)        
			sei(); // Re-enable interrupts
    
		}
    
	}

}

/* storePulse(index, us, inRangeLow, inRangeHigh) - map input values to PPM durations (min/max) and store pulse */

void storePulse(byte index, int inValue, int inRangeLow, int inRangeHigh) {

	int mappedPulse = map(inValue, inRangeLow, inRangeHigh, PPM_MIN_PULSE, PPM_MAX_PULSE); // Map input value to pulse width
	pulses[(index * 2) + 1] = mappedPulse; // Store new pulse width

}


/* handleControlUpdate() - Handle updates to the controls */

void handleControlUpdate() {
  
	int x;
	storePulse(0, servos[0], 0, 254);  // Write ESC #1
	storePulse(1, servos[1], 0, 254);  // Write ESC #2
	// Write all remaning servo pulses
	for(x = 2 ; x < SERVO_COUNT ; x++) {

		storePulse(x, servos[x], 0, 180);

	}
	if(buttons[4] > 0) {
    
		navlightEnabled = true;  // enable navlight if button 5 is on
    
	} else {
    
		navlightEnabled = false; // otherwise disable it
    
	}
  
	#if DEBUG_LEVEL == 5
	for(x = 0 ; x < PPM_PULSES ; x++) {

		dbgMsg.length = snprintf(dbgMsg.messageBuffer, MSG_BUFFER_SIZE, "----Pulse[%d]: %d-", x, pulses[x]); // Build debug message
		writeXBeeMessage(&dbgMsg, MSG_TYPE_DBG);                                              // Write debug message
    
	}
	#endif
  
}

/* Write a message back to the XBee port */

void writeXBeeMessage(messageState *msg, unsigned char msgType) {
 
	msg->messageBuffer[0] = MSG_BEGIN;                                                         // Message construction 
	msg->messageBuffer[1] = msgType;                                                           // Specify the message type
	msg->messageBuffer[2] = msg->length;                                                       // Message size
	msg->messageBuffer[3] = generateChecksum(msg->messageBuffer, MSG_HEADER_SIZE - 1);         // Header checksum
	msg->messageBuffer[msg->length - 1] = generateChecksum(msg->messageBuffer, msg->length - 1); // Fill in our checksum for the whole message
   
	Serial.write(msg->messageBuffer, msg->length);  // Write out the message
     
}

/* Write a message back to the PPZ port */

void writePPZMessage(messageState *msg) {
   
	int x;
	
	msg->messageBuffer[msg->length - 1] = '\0'; // End-of-string for last character replaces checksum

	for(x = 0 ; x < msg->length - MSG_HEADER_SIZE; x++) {
	
		msg->messageBuffer[x] = msg->messageBuffer[x + MSG_HEADER_SIZE]; // Shift everything MSG_HEADER_SIZE to the left to drop the header
	
	}

	Serial.write(msg->messageBuffer, msg->length - MSG_HEADER_SIZE);  // Write out the message, minus the header size
     
}

/* Init our timer */

void initTimer() {
  
	cli();   
	DDRD  &= B11011111; // Disable output on OC1A      
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
