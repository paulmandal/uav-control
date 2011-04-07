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

#define DEBUG_LEVEL 3   // 1 - Messaging debugging
                        // 2 - Servo / pin output
                        // 3 - Signal continuity debugging (light 4 stays on if signal is ever lost)
                        // 4 - PPM registers
                        // 5 - PPM pulse values
#define DEBUG_PIN1  4   // Pin for debug signaling   
                     

#define VERSION_MAJOR 2     // Major version #
#define VERSION_MINOR 9     // Minor #
#define VERSION_MOD   3     // Mod #

#define MSG_BEGIN     0xFF                    // Begin of control message indicator byte
#define MSG_TYPE_CTRL 0x01                    // Control update message type indicator
#define MSG_TYPE_CFG  0x02		      // Configuration update
#define MSG_TYPE_PPZ  0x03                    // Message from PPZ
#define MSG_TYPE_SYNC 0xFE                    // Sync message type indicator
#define MSG_BUFFER_SIZE 256 		      // Message buffer size in bytes
#define MSG_HEADER_SIZE 4     		      // Message header size in bytes
#define CMDS_PER_ACK  50                      // Client will assume lost signal if we send this many commands without an ack
#define MSG_INTERVAL  20                      // Control message update interval (20ms)
#define LOST_MSG_THRESHOLD (MSG_INTERVAL * 3) // How long without legit msg before lostSignal gets set

#define SERVO_COUNT 8       // # of servos
#define BUTTON_COUNT 12     // # of buttons on controller

#define PPM_PIN 23          // Pin to output PPM signal on
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

/* Various varibles to hold state info */

unsigned int servos[SERVO_COUNT];        // store servo states
unsigned int buttons[BUTTON_COUNT];      // store button states

boolean lostSignal = true;               // lostSignal state
unsigned long lastMsgTime = 0;           // Time of last legit message

unsigned long navlightInterval = NAVLIGHT_INTERVAL; // Interval for navigation lights
unsigned long navlightLastTime = 0;      // Navigation light last 
boolean navlightState = false;           // Navigation light LED state
boolean navlightEnabled = false;         // Enable/disable navigation lights

unsigned long lastStatusLEDTime = 0;     // Time of last status LED change
unsigned long statusLEDInterval = STATUS_INTERVAL_OK; // Current status LED toggle interval
boolean statusLEDState = false;          // Status LED state

volatile byte currentPulse = 0;        // The pulse being sent
boolean  ppmON = false;
unsigned int pulses[PPM_PULSES];        // PPM pulses

messageState xbeeMsg;

int commandsSinceLastAck = 0;

/* Arduino is racist against function prototypes 

void initControlState();
void initPPM();
void initOutputs();
boolean initMessage(messageState message);
void updateStatusLED();
void updateNavigationLights();
boolean checkMessages(messageState msg);
boolean testMessage(unsigned char *message, int length);
void processMessage(unsigned char *message, int length);
void checkSignal();
void storePulse(byte index, int inValue, int inRangeLow, int inRangeHigh);*/

/* Setup function */

void setup() {

  delay(100);                // Wait 100ms since some of our timers use millis() and it starts at 0.  They expect more from it.  They're disappoint.
                             // This also gives PuTTY (using for diag on 2nd UART) time to open and connect
  randomSeed(analogRead(0)); // Seed our random number gen with an unconnected pins static
  initControlState();        // Initialise control state
  initOutputs();             // Initialise outputs
  initPPM();                 // Set default PPM pulses
  initMessage(xbeeMsg);      // Init our message header
  Serial.begin(115200);      // Open XBee/GCS Serial
  Serial.flush();
  #if DEBUG_LEVEL > 0
  Serial1.begin(115200);      // Open PPZ port as debug
  Serial1.println();  // Give us a little space in the output terminal / monitor
  Serial1.println();
  Serial1.println();
  Serial1.print("joystick2ppm version ");  // Output version info
  Serial1.print(VERSION_MAJOR);
  Serial1.print(".");
  Serial1.print(VERSION_MINOR);
  Serial1.print(".");
  Serial1.print(VERSION_MOD);
  Serial1.println("...");
  Serial1.println("Open for debugging mode..");  // Let them know we're ready
  #endif
  
  #if DEBUG_LEVEL == 3
  pinMode(DEBUG_PIN1, OUTPUT);  //  DEBUG - Pin will light permanently if signal is lost
  #endif

}

/* Loop function */

void loop() {

  int x;
  updateStatusLED();        // Check if we need to toggle the status LED
  updateNavigationLights(); // Update Navigation lights

  for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) { // checkMessages() should be run with a much higher frequency than the LED updates or checkSignal()
  
    checkMessages(xbeeMsg);          // Check for incoming messages

  }

  checkSignal();            // Check if the signal is still good

}

/* Function definitions */

/* initMessage() - Initialise message */

boolean initMessage(messageState message) {

	message.readBytes = 0;
	message.length = MSG_HEADER_SIZE; // Init message.length as header length size
	if((message.messageBuffer = (unsigned char*)calloc(MSG_BUFFER_SIZE, sizeof(char))) != NULL) {
	
		return true; // calloc() worked
	
	} else {
	
		return false; // calloc() failed
	
	} 
	
}

/* initControlState() - Zeroes out everythang */

void initControlState() {

int x;

  // Zero out all buttons and servos

  for(x = 0 ; x < SERVO_COUNT ; x++) {

    servos[x] = 0;

  }

  for(x = 0 ; x < BUTTON_COUNT	; x++) {

    buttons[x] = 0;

  }

}

/* initPPM - What do you think? */

void initPPM() {

  int x;
  int midPPMPulse = (PPM_MIN_PULSE + PPM_MAX_PULSE) / 2;  
  pinMode(PPM_PIN,OUTPUT);  // Setup PPM output Pin

  for (x = 0 ; x < (SERVO_COUNT + 1) ; x++) {
    
    pulses[x * 2] = PPM_HIGH_PULSE + x;  // DEBUG
    pulses[(x * 2) + 1] = midPPMPulse; // Set all PPM pulses to halfpulse
    
  }
  pulses[PPM_PULSES - 1] = PPM_SYNC_PULSE; // Sync pulse is before 0 length pulse

  #if DEBUG_LEVEL == 5
  for(x = 0 ; x < PPM_PULSES ; x++) {
    
    Serial1.print("Pulse[");
    Serial1.print(x);
    Serial1.print("]: ");
    Serial1.println(pulses[x]);
    
  }
  #endif
  currentPulse = 0; // init currentPulse

}

/* initOutputs() - Set output pins up */

void initOutputs() {
  
  pinMode(STATUS_LED_PIN, OUTPUT); // Status LED Pin
  pinMode(NAVLIGHT_PIN, OUTPUT);   // Navlight LED(s) Pin
  
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

/* checkMessages() - Check for and handle any incoming messages */

boolean checkMessages(messageState msg) {

	unsigned char testByte = 0x00;
	
	if(msg.readBytes == MSG_HEADER_SIZE) {

		int x;	
		// Finished reading the message header, check it
		if(testChecksum(msg.messageBuffer, msg.readBytes)) { // Checksum was good

	
			msg.length = msg.messageBuffer[2];  // 0 - MSG_BEGIN, 1 - MSG_TYPE, 2 - MSG_LENGTH


		} else {			

			for(x = 0 ; x < (MSG_HEADER_SIZE - 1) ; x++) { // Shift all message characters to the left, drop the first one

				msg.messageBuffer[x] = msg.messageBuffer[x + 1];

			}

			msg.readBytes--; // Decrement byte count and chuck the first byte, this will allow us to reprocess the other 3 bytes in case we are desynched with the message sender

		}

	} 

	if(msg.readBytes < msg.length) { // Message is not finished being read

		if(Serial.available() > 0) {

			testByte = Serial.read();  // Read our byte		

			#if DEBUG_LEVEL == 1
			Serial1.print("BYTE[");
			Serial1.print(msg.readBytes);
			Serial1.print("/");
			Serial1.print(msg.length);
			Serial1.print(" - CSLA:");
			Serial1.print(commandsSinceLastAck);
			Serial1.print("]: ");
			Serial1.println(testByte, HEX);
			#endif		

			msg.messageBuffer[msg.readBytes] = testByte; // Add the new byte to our message buffer
			msg.readBytes++;			  // Increment readBytes
	
			return true;
	
		} else {

			return false;
	
		}

	} else { // Message is finished, process it

		if(testChecksum(msg.messageBuffer, msg.length)) { // Checksum passed, process message..  If the checksum failed we can assume corruption elsewhere since the header was legit

			processMessage(msg.messageBuffer, msg.length);

		} 

		int x;	

		// Clear out message so it's ready to be used again	
		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

			msg.messageBuffer[x] = '\0';

		}
		msg.readBytes = 0;            // Zero out readBytes
		msg.length = MSG_HEADER_SIZE; // Set message length to header length
		
		return 1;

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
      DDRD  &= B11011111;                              // Disable output on OC1A
      TIMSK1 = B00000000;                              // Disable interrupt on compare match
      TCCR1A = B00000000;                              // Disable fast PWM
      TCCR1B = B00000000;                              // Disable fast PWM, clock, and prescaler
      #if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 3
      Serial1.println("Lost signal due to lastMsgTime timeout!");
      #endif
      #if DEBUG_LEVEL == 3
      Serial1.print("commandsSinceLastAck: ");
      Serial1.print(commandsSinceLastAck);
      Serial1.print("currentTime: ");
      Serial1.print(currentTime);
      Serial1.print("lastMsgTime: ");
      Serial1.print(lastMsgTime);
      Serial1.print("diff: ");
      Serial1.print(currentTime - lastMsgTime);
      Serial1.print(" > ");
      Serial1.println(LOST_MSG_THRESHOLD);
      #endif
      statusLEDInterval = STATUS_INTERVAL_SIGNAL_LOST; // Set status LED interval to signal lost
      #if DEBUG_LEVEL == 3
      digitalWrite(DEBUG_PIN1, HIGH);
      #endif
      sei(); // Re-enable interrupts
      
    }

  } else {
    
    if(!ppmON) {  // Restart PPM since it was off

        cli();  // This shouldn't get interrupted since PPM is off but just to be safe..

        #if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 3
        Serial1.println("Restarting PPM, lostSignal = false");
        #endif        

        DDRD  |= B00100000;                     // Enable output on OC1A
        TIMSK1 = B00000010;                     // Interrupt on compare match with OCR1A        

    	ppmON = true; 
	TCNT1 = 0;				// Zero out counter, shouldn't matter but just in case                          
        OCR1A = pulses[0];                      // Set OCR1A to pulse[0], this won't actually matter until we set TCCR1A and TCCR1B at the end to enable fast PWM
	currentPulse = 1;                       // Set currentPulse to 1 since there will be no ISR() call to increment it

        TCCR1A = B11000000;                     // CTC, set OC1A HIGH on match
        TCCR1B = B00001000;                     // CTC, clock disabled, OCR1A has our value for pulse[0] but will never be reached by TCNT1 'coz no clock

        TCCR1C = B10000000;                     // Force match, should set pin high, WILL NOT generate ISR() call        

        TCCR1A = B01000011;                     // Fast PWM mode, will generate ISR() when it reaches OCR1A (pulse[0]), thus starting the PPM signal
        TCCR1B = B00011010;                     // Fast PWM, plus 8 prescaler (bit 2, disabled until PPM on), 16bits holds up to 65535, 8 PS puts our counter into useconds (16MHz / 8 * 2 = 1MHz)
	statusLEDInterval = STATUS_INTERVAL_OK; // Set our status LED interval to OK
        sei(); // Re-enable interrupts
    
    }
    
  }

}

/* storePulse(index, us, inRangeLow, inRangeHigh) - map input values to PPM durations (min/max) and store pulse */

void storePulse(byte index, int inValue, int inRangeLow, int inRangeHigh) {

  unsigned int mappedPulse = map(inValue, inRangeLow, inRangeHigh, PPM_MIN_PULSE, PPM_MAX_PULSE); // Map input value to pulse width
  pulses[(index * 2) + 1] = mappedPulse; // Store new pulse width

}


/* testChecksum(message, length) - Test if the last byte checksum is good */

int testChecksum(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	#if DEBUG_LEVEL == 1
	Serial1.print("CHKMSG: ");
	#endif

	for(x = 0 ; x < length ; x++) {

		#if DEBUG_LEVEL == 1
		Serial1.print((unsigned int)message[x], HEX);
		Serial1.print(" ");
		#endif
                checksum = checksum ^ (unsigned int)message[x];  // Test this message against its checksum (last byte)

	}
	#if DEBUG_LEVEL == 1
	Serial1.print("CHK: ");
	Serial1.println(checksum, HEX);
        Serial1.print("CSLA: ");
        Serial1.println(commandsSinceLastAck);
	#endif

	if(checksum == 0x00) {

		return true;  // Checksum passed!

	} else {

		return false;

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

        int x;
	unsigned char msgType = message[1];

	#if DEBUG_LEVEL == 1
        Serial1.print("CSLA: ");
        Serial1.println(commandsSinceLastAck);
	#endif

	if(msgType == MSG_TYPE_SYNC) {  // Handle the message, since it got past checksum it has to be legit

		 sendAck(); // Send ACK for the SYNC signal

	} else if(msgType == MSG_TYPE_CTRL) { // Handle updating the controls

 		lastMsgTime = millis(); // Only command messages count for this
        	lostSignal = false;     // Message was legit, update lostSignal and lastMsgTime
		/* MSG structure - [BEGIN_MSG] [MSG_TYPE] [SERVOS] [BUTTONS] [CHECKSUM]
	         * BEGIN_MSG - 1 byte  - 1 byte msg marker
		 * MSG_TYPE  - 1 byte  - 1 byte msg type marker
	         * SERVOS    - 8 bytes - 1 byte per servo
	         * BUTTONS   - 3 bytes - 2 bits per pin (allow more than on/off, e.g. 3-pos switch)
	         * CHECKSUM  - 1 byte  - 1 byte XOR checksum
	         */
          
	        for(x = 0 ; x < SERVO_COUNT ; x++) {

	          servos[x] = message[x + 2]; // Set latest servo values from msg

	        }

	        for(x = 0 ; x < 3 ; x++) {  // This loop handles 4 buttons at once since each uses 2 bits and we read in 1 byte (2 bits * 4 = 8 bits = 1 byte)

	          buttons[(x * 4)] = (message[x + 10] & B11000000) >> 6;     // Bitwise and against our byte to strip away other button values, then bitshift to 0th and 1st positions
	          buttons[(x * 4) + 1] = (message[x + 10] & B00110000) >> 4; // Same, you can see the bitmask shift to the right as we work out way down the byte
		  buttons[(x * 4) + 2] = (message[x + 10] & B00001100) >> 2; // Same
	          buttons[(x * 4) + 3] = (message[x + 10] & B00000011);      // No bitshift here since our bits are already in 0th and 1st pos.

	        }

                handleControlUpdate();

	} else if(msgType == MSG_TYPE_PPZ) { // Handle PPZ message
	} else if(msgType == MSG_TYPE_CFG) { // Handle configuration message
	}

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
    
    Serial1.print("Pulse[");
    Serial1.print(x);
    Serial1.print("]: ");
    Serial1.println(pulses[x]);
    
  }
  #endif
  
}

/* sendAck() - Send SYNC acknowledgement message */

void sendAck() {

	unsigned char *msgSync;
        unsigned int checksum;
	int x, msgSize;

        msgSize = random(10, 30); // Sync message is between 10 and 30 chars
        
        msgSync = (unsigned char*)calloc(msgSize, sizeof(char)); // Allocate memory for sync msg
        
        #if DEBUG_LEVEL == 1	
	Serial1.println("Sending SYNC ACK");
	#endif 

	msgSync[0] = MSG_BEGIN;  // Use our msg buffer to write back a sync reply
	msgSync[1] = MSG_TYPE_SYNC;  // Sync reply will have same format (msgBegin, msgType, random chars, checksum)
        msgSync[2] = msgSize; // Message length
        msgSync[3] = generateChecksum(msgSync, 3); // Header checksum
        for(x = 4 ; x < msgSize ; x++) { 

		msgSync[x] = (unsigned char)random(0, 254); // Fill all but the last character with random bytes

	}
        msgSync[msgSize - 1] = generateChecksum(msgSync, msgSize - 1); // Store our message checksum
	Serial.write(msgSync, msgSize);     // Send the sync ACK
        Serial.flush();                          // Flush the serial buffer since it may be full of garbage
        commandsSinceLastAck = 0;               // Set commandsSinceLastAck to 0
        
        free(msgSync); // Deallocate memory for sync msg

}

/* ISR - TIMER1_COMPAT_Vect, generates the PPM signal */

ISR(TIMER1_COMPA_vect) {

  OCR1A = pulses[currentPulse];    // Set OCR1A compare register to our next pulse
  currentPulse++;                  // Increment the pulse counter
  if(currentPulse >= PPM_PULSES) { // If the pulse counter is too high reset it
    
    currentPulse = 0;
    
  }
 
}
