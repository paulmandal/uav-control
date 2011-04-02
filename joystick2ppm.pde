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

/* This is the defining moment of the file */

#define DEBUG_LEVEL 3   // 1 - Messaging debugging
                        // 2 - Servo / pin output
                        // 3 - Signal continuity debugging (light 4 stays on if signal is ever lost)
                        // 4 - PPM Debugging (output PPM info)
#define DEBUG_PIN1  4   // Pin for debug signaling   
                     

#define VERSION_MAJOR 2     // Major version #
#define VERSION_MINOR 5     // Minor #
#define VERSION_MOD   0     // Mod #

#define MSG_SIZE_CTRL 14                      // Length of control update messages
#define MSG_SIZE_SYNC 14		      // Length of sync messages
#define MSG_SIZE_PPZ  64                      // Length of message from PPZ
#define MSG_SIZE_CFG  64		      // Length of configuration message
#define MSG_BEGIN     0xFF                    // Begin of control message indicator byte
#define MSG_TYPE_CTRL 0x01                    // Control update message type indicator
#define MSG_TYPE_CFG  0x02		      // Configuration update
#define MSG_TYPE_PPZ  0x03                    // Message from PPZ
#define MSG_TYPE_SYNC 0xFE                    // Sync message type indicator
#define MSG_BUFFER_SIZE 128 		      // Message buffer size in bytes
#define MSG_MIN_READ_INTERVAL 1               // 1ms per message byte because Serial.available() lies and gives us old bytes! :(
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
unsigned long statusLEDInterval = 0;     // Current status LED toggle interval
boolean statusLEDState = false;          // Status LED state

volatile byte currentPulse = 0;        // The channel being pulsed
boolean  ppmON = false;
unsigned int pulses[PPM_PULSES];        // PPM pulses

unsigned char inMsg[MSG_BUFFER_SIZE];    // Incoming message buffer
int msgWaitingBytes = 0;     // Message waiting byte count
int msgReadBytes = 0;
boolean gotMsgBegin = false; // Mark whether or not we're currently reading a message
boolean gotMsgType = false;  // Mark whether we have a message type

int commandsSinceLastAck = 0;

/* Function prototypes */

void initControlState();
void initPPM();
void initOutputs();
void updateStatusLED();
void updateNavigationLights();
void checkMessages();
boolean testMessage(unsigned char *message, int length);
void processMessage(unsigned char *message, int length);
void checkSignal();
void storePulse(byte targetChannel, int inValue, int inRangeLow, int inRangeHigh);

/* Setup function */

void setup() {
    
  delay(100);                // Wait 100ms since some of our timers use millis() and it starts at 0.  They expect more from it.  They're disappoint.
                             // This also gives PuTTY (using for diag on 2nd UART) time to open and connect
  randomSeed(analogRead(0)); // Seed our random number gen with an unconnected pins static
  initControlState();        // Initialise control state
  initOutputs();             // Initialise outputs
  initPPM();                 // Set default PPM pulses
  Serial.begin(115200);      // Open XBee/GCS Serial
  Serial.flush();
  #if DEBUG_LEVEL == 1 || DEBUG_LEVEL == 2 || DEBUG_LEVEL == 4
  Serial1.begin(115200);      // Open PPZ port as debug
  Serial1.print("joystick2ppm version ");
  Serial1.print(VERSION_MAJOR);
  Serial1.print(".");
  Serial1.print(VERSION_MINOR);
  Serial1.print(".");
  Serial1.print(VERSION_MOD);
  Serial1.println("...");
  Serial1.println("Open for debugging mode..");
  #endif
  
  #if DEBUG_LEVEL == 3
  pinMode(DEBUG_PIN1, OUTPUT);  //  DEBUG - Pin will light permanently if signal is lost
  #endif

}

/* Loop function */

void loop() {

  updateStatusLED();        // Check if we need to toggle the status LED
  updateNavigationLights(); // Update Navigation lights
  checkMessages();          // Check for incoming messages
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
    
    pulses[x * 2] = PPM_HIGH_PULSE;
    pulses[(x * 2) + 1] = midPPMPulse; // Set all PPM pulses to halfpulse
    
  }
  pulses[PPM_PULSES - 1] = PPM_SYNC_PULSE; // Sync pulse is before 0 length pulse

  #if DEBUG_LEVEL == 4
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

void checkMessages() {

  int x;
  unsigned char testByte;
  unsigned long currentTime = millis();
  
  if(Serial.available() > 0) {  // All of our bytes are here (either this is 1 or a message size), so process them

    testByte = Serial.read();

    if(!gotMsgBegin) { // We're waiting for a message begin marker, so check for one

	if(testByte == MSG_BEGIN) {  // Found a message begin marker

		#if DEBUG_LEVEL == 1
		Serial1.print("BYTE[1/");
		Serial1.print(msgWaitingBytes);
		Serial1.print("]: ");
		Serial1.print(testByte, HEX);
		Serial1.println("  MSG BEGIN");
		#endif
		inMsg[0] = testByte;
		gotMsgBegin = true;
                msgReadBytes = 1;
                msgWaitingBytes = 2;
                
	} // Discard useless byte
	
        #if DEBUG_LEVEL == 1	
        else {
          
    	  Serial1.print("BYTE[0/");
	  Serial1.print(msgWaitingBytes);
	  Serial1.print("]: ");
	  Serial1.print(testByte, HEX);
	  Serial1.println("  JUNK BYTE");	
        }
	#endif

    } else if(!gotMsgType) {  // We're waiting for a message type marker, grab this one

	inMsg[1] = testByte;

	#if DEBUG_LEVEL == 1
	Serial1.print("BYTE[2/");
	Serial1.print(msgWaitingBytes);
	Serial1.print("]: ");
	Serial1.print(testByte, HEX);
	Serial1.print("  MSG TYPE");
	#endif

	gotMsgType = true;
        msgReadBytes = 2;

	if(testByte == MSG_TYPE_SYNC) { // Message is a sync message, handle it that way

		#if DEBUG_LEVEL == 1
		Serial1.println("-SYNC");
		#endif
		msgWaitingBytes = MSG_SIZE_SYNC;
		
	} else if(testByte == MSG_TYPE_CTRL) {

		#if DEBUG_LEVEL == 1
		Serial1.println("-CTRL");
		#endif
		msgWaitingBytes = MSG_SIZE_CTRL;

	} else if(testByte == MSG_TYPE_PPZ) {

		#if DEBUG_LEVEL == 1
		Serial1.println("-PPZ");
		#endif
		msgWaitingBytes = MSG_SIZE_PPZ;

	} else if(testByte == MSG_TYPE_CFG) {

		#if DEBUG_LEVEL == 1
		Serial1.println("-CFG");
		#endif
		msgWaitingBytes = MSG_SIZE_CFG;
	
	} else { // Not a valid message type?  Ignore this garbage

		#if DEBUG_LEVEL == 1
		Serial1.println("-INVALID");
		#endif
		gotMsgBegin = false;  // Set ready for next message
		gotMsgType = false;
		msgWaitingBytes = 1;

	}

    } else { // We have our msgBegin and msgType markers, read the rest of the message into our buffer and handle it

        inMsg[msgReadBytes] = testByte;
	#if DEBUG_LEVEL == 1
	Serial1.print("BYTE[");
	Serial1.print(msgReadBytes);
	Serial1.print("/");
	Serial1.print(msgWaitingBytes);
	Serial1.print("]: ");
	Serial1.print(testByte, HEX);
	Serial1.println("  DATA");
	#endif
        msgReadBytes++;

        if(msgReadBytes >= msgWaitingBytes) {  // Message is done reading

	if(testMessage(inMsg, msgWaitingBytes)) {  // Test the message checksum

		commandsSinceLastAck++;

		processMessage(inMsg, msgWaitingBytes); // Execute or process the message

		if(commandsSinceLastAck > CMDS_PER_ACK) {

			sendAck();  // Let the controller know we're alive

		}

	}

	gotMsgBegin = false;  // Set ready for next message
	gotMsgType = false;  
  	msgWaitingBytes = 1;

        }

    }

  }
 
}

/* checkSignal() - Check the signal state and make necessary updates */

void checkSignal() {

  unsigned long currentTime = millis(); // get current time
  if((currentTime - lastMsgTime) > LOST_MSG_THRESHOLD) {

    if(!lostSignal) {
      
      #if DEBUG_LEVEL == 1
      Serial1.println("Lost signal due to lastMsgTime timeout!");
      #endif
      lostSignal = true;                               // If we haven't received a message in > LOST_MSG_THRESHOLD set lostSignal
      ppmON = false;                                   // Disable PPM
      DDRD  &= B11011111;                              // Disable output on OC1A
      TIMSK1 = B00000000;                              // Disable interrupt on compare match
      TCCR1A = B00000000;                              // Disable fast PWM
      TCCR1B = B00000000;                              // Disable fast PWM, clock, and prescaler
      statusLEDInterval = STATUS_INTERVAL_SIGNAL_LOST; // Set status LED interval to signal lost
      #if DEBUG_LEVEL == 3
      digitalWrite(DEBUG_PIN1, HIGH);
      #endif
      
    }

  } else {
    
    if(!ppmON) {  // Restart PPM since it was off

        #if DEBUG_LEVEL == 1
        Serial1.println("Restarting PPM");
        #endif    
    	ppmON = true;                           // Turn ppm LOW (since the signal is probably off)
	currentPulse = 0;                       // Set our currentPulse to the last channel
        OCR1A = pulses[currentPulse];           // Set our current pulse length
        DDRD  |= B00100000;                     // Enable output on OC1A
        TIMSK1 = B00000010;                     // Interrupt on compare match with OCR1A
        TCCR1A = B01000011;                     // Fast PWM
        TCCR1B = B00011010;                     // Fast PWM, plus 8 prescaler (bit 2, disabled until PPM on), 16bits holds up to 65535, 8 PS puts our counter into useconds (16MHz / 8 * 2 = 1MHz)
	statusLEDInterval = STATUS_INTERVAL_OK; // Set our status LED interval to OK
    
    }
    
  }

}

/* storePulse(targetChannel, us, inRangeLow, inRangeHigh) - map input values to PPM durations (min/max) and store pulse */

void storePulse(byte targetChannel, int inValue, int inRangeLow, int inRangeHigh) {

  unsigned int mappedPulse = map(inValue, inRangeLow, inRangeHigh, PPM_MIN_PULSE, PPM_MAX_PULSE); // Map input value to pulse width
  pulses[(targetChannel * 2) + 1] = mappedPulse; // Store new pulse width

}

/* testMessage(message, length) - Test if the last byte checksum is good */

boolean testMessage(unsigned char *message, int length) {

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
  // Write all remaning servo channels
  for(x = 2 ; x < SERVO_COUNT ; x++) {

    storePulse(x, servos[x], 0, 180);

  }
  if(buttons[4] > 0) {
    
    navlightEnabled = true;  // enable navlight if button 5 is on
    
  } else {
    
    navlightEnabled = false; // otherwise disable it
    
  }
  
  #if DEBUG_LEVEL == 4
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

	unsigned char msgSync[MSG_SIZE_SYNC];
        unsigned int checksum;
	int x;

	#if DEBUG_LEVEL == 1	
	Serial1.println("Sending SYNC ACK");
	#endif 

	msgSync[0] = MSG_BEGIN;  // Use our msg buffer to write back a sync reply
	msgSync[1] = MSG_TYPE_SYNC;  // Sync reply will have same format (msgBegin, msgType, random chars, checksum)
	for(x = 2 ; x < (MSG_SIZE_SYNC - 1) ; x++) { 

		msgSync[x] = (unsigned char)random(0, 254); // Fill all but the last character with random bytes

	}
	checksum = 0x00;
	for(x = 0 ; x < (MSG_SIZE_SYNC - 1) ; x++) {

		checksum = checksum ^ (unsigned int)msgSync[x];  // Generate our checksum for the sync msg

	}
	msgSync[MSG_SIZE_SYNC - 1] = checksum & 0xFF;  // Store the checksum
	Serial.write(msgSync, MSG_SIZE_SYNC);     // Send the sync ACK
        Serial.flush();                          // Flush the serial buffer since it may be full of garbage
        commandsSinceLastAck = 0;               // Set commandsSinceLastAck to 0

}

/* ISR - TIMER1_COMPAT_Vect, generates the PPM signal */

ISR(TIMER1_COMPA_vect) {

  OCR1A = pulses[currentPulse];    // Set OCR1A compare register to our next pulse
  currentPulse++;                  // Increment the pulse counter
  if(currentPulse >= PPM_PULSES) { // If the pulse counter is too high reset it
    
    currentPulse = 0;
    
  }
 
}
