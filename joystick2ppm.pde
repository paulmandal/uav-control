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

#define VERSION_MAJOR 2     // Major version #
#define VERSION_MINOR 1     // Minor #
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
#define MSG_INTERVAL 20                       // Control message update interval (20ms)
#define LOST_MSG_THRESHOLD (MSG_INTERVAL * 3) // How long without legit msg before lostSignal gets set

#define SERVO_COUNT 8       // # of servos
#define BUTTON_COUNT 12     // # of buttons on controller

#define PPM_PIN 23          // Pin to output PPM signal on
#define PPM_MIN_PULSE 1000  // Min pulse length (1ms)
#define PPM_MAX_PULSE 2000  // Max pulse length (2ms)
#define PPM_HIGH_PULSE 200  // Delay between pulses (200us)
#define PPM_FREQUENCY 20000 // Frequency of PPM frame (20ms)
#define PPM_SYNC_PULSE (PPM_FREQUENCY - (SERVO_COUNT * (((PPM_MAX_PULSE + PPM_MIN_PULSE) / 2) + PPM_HIGH_PULSE))) // Duration of sync pulse

#define STATUS_LED_PIN 0
#define STATUS_INTERVAL_SIGNAL_LOST 100 // Toggle every 100ms
#define STATUS_INTERVAL_OK 1000         // Toggle every 1s

#define NAVLIGHT_PIN 12                 // Navigation light pin
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

volatile byte currentChannel = 0;        // The channel being pulsed
enum ppmStates { ppmOFF, ppmHIGH, ppmLOW };
enum ppmStates ppmState = ppmOFF;        // PPM status
unsigned int channels[SERVO_COUNT];      // Servo channels

unsigned char inMsg[128];    // Incoming message buffer
int msgWaitingBytes = 1;     // Message waiting byte count
boolean gotMsgBegin = false; // Mark whether or not we're currently reading a message
boolean gotMsgType = false;  // Mark whether we have a message type

/* Function prototypes */

void initControlState();
void initTimer();
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

  randomSeed(analogRead(0)); // Seed our random number gen with an unconnected pins static
  initControlState();        // Initialise control state
  initOutputs();             // Initialise outputs
  initPPM();                 // Enable PPM 
  initTimer();               // Turn on Timer
  Serial.begin(38400);       // Open XBee/GCS Serial
  Serial.flush();

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

  for (x = 0 ; x < SERVO_COUNT ; x++) {
    
    channels[x] = midPPMPulse; // Set all PPM pulses to halfpulse
    
  }

  currentChannel = 0; // init currentChannel

}

/* initTimer() - Set up ATmega timers */

void initTimer() {

  // Timer1, used to make PPM waveform real nice like

  TIMSK1 = B00000010; // Interrupt on compare match with OCR1A
  TCCR1A = B00000011; // Fast PWM
  TCCR1B = B00011010; // Fast PWM, plus 8 prescaler, 16bits holds up to 65535, 8 PS puts our counter into useconds (16MHz / 8 * 2 = 1MHz)
  
  // Timer2, used to update PPM pulses based on servo states

  TIMSK2 = B00000010; // Interrupt on compare match with OCR2A
  TCCR2A = B00000010; // CTC mode
  TCCR2B = B00000111; // 1024 prescaler because 8bits does not store a lot
  OCR2A  = 78; // ~100hz == (16MHz / (2 * 1024 * 78))
  
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
  if(Serial.available() >= msgWaitingBytes) {  // All of our bytes are here (either this is 0 or a message size), so process them

    if(!gotMsgBegin) { // We're waiting for a message begin marker, so check for one

	testByte = Serial.read();
	if(testByte == MSG_BEGIN) {  // Found a message begin marker

		inMsg[0] = testByte;
		gotMsgBegin = true;

	} // Discard useless byte

    } else if(!gotMsgType) {  // We're waiting for a message type marker, grab this one

	testByte = Serial.read();
	inMsg[1] = testByte;
	gotMsgType = true;
	if(testByte == MSG_TYPE_SYNC) { // Message is a sync message, handle it that way

		msgWaitingBytes = MSG_SIZE_SYNC - 2;  // Subtract 2 since we already read two bytes off the buffer
		
	} else if(testByte == MSG_TYPE_CTRL) {

		msgWaitingBytes = MSG_SIZE_CTRL - 2;

	} else if(testByte == MSG_TYPE_PPZ) {

		msgWaitingBytes = MSG_SIZE_PPZ - 2;

	} else if(testByte == MSG_TYPE_CFG) {

		msgWaitingBytes = MSG_SIZE_CFG - 2;
	
	} else { // Not a valid message type?  Ignore this garbage

		gotMsgBegin = false;
		gotMsgType = false;
		msgWaitingBytes = 1;

	}

    } else { // We have our msgBegin and msgType markers, read the rest of the message into our buffer and handle it

	for(x = 0 ; x < msgWaitingBytes ; x++) {  // Don't overwrite the msgBegin or msgType bytes

	    inMsg[x + 2] = Serial.read();  // Read the rest of the message into our buffer

	}

	gotMsgBegin = false;
	gotMsgType = false;
	msgWaitingBytes = 1;

	if(testMessage(inMsg, msgWaitingBytes + 2)) {  // Test the message checksum

		lastMsgTime = millis();  
        	lostSignal = false;     // Message was legit, update lostSignal and lastMsgTime

		processMessage(inMsg, msgWaitingBytes + 2); // Execute or process the message

	}

    }

  }

}

/* checkSignal() - Check the signal state and make necessary updates */

void checkSignal() {

  unsigned long currentTime = millis(); // get current time
  if((currentTime - lastMsgTime) > LOST_MSG_THRESHOLD) {

    lostSignal = true;                               // If we haven't received a message in > LOST_MSG_THRESHOLD set lostSignal
    ppmState = ppmOFF;                               // Disable PPM
    statusLEDInterval = STATUS_INTERVAL_SIGNAL_LOST; // Set status LED interval to signal lost

  }

}

/* storePulse(targetChannel, us, inRangeLow, inRangeHigh) - map input values to PPM durations (min/max) and store pulse */

void storePulse(byte targetChannel, int inValue, int inRangeLow, int inRangeHigh) {

  unsigned int mappedPulse = map(inValue, inRangeLow, inRangeHigh, PPM_MIN_PULSE, PPM_MAX_PULSE); // Map input value to pulse width
  channels[targetChannel] = mappedPulse; // Store new pulse width

}

/* testMessage(message, length) - Test if the last byte checksum is good */

boolean testMessage(unsigned char *message, int length) {

	unsigned int checksum = 0x00;
	int x;

	for(x = 0 ; x < length ; x++) {

		checksum = checksum ^ (unsigned int)message[x];

	}

	if(checksum == 0x00) {

		return true;  // Checksum passed!

	} else {

		return false;

	}

}

/* processMessage(message, length) - Do whatever the message tells us to do */

void processMessage(unsigned char *message, int length) {

	unsigned char msgType = message[1];
	unsigned char msgSync[MSG_SIZE_SYNC];

	if(msgType == MSG_TYPE_SYNC) {  // Handle the message, since it got past checksum it has to be legit

		 msgSync[0] = MSG_BEGIN;  // Use our msg buffer to write back a sync reply
		 msgSync[1] = MSG_TYPE_SYNC;  // Sync reply will have same format (msgBegin, msgType, random chars, checksum)
		 for(x = 2 ; x < (MSG_SIZE_SYNC - 1) ; x++) { 

			msgSync[x] = (unsigned char)random(0, 255); // Fill all but the last character with random bytes

		 }
		 checksum = 0x00;
		 for(x = 0 ; x < (MSG_SIZE_SYNC - 1) ; x++) {

			checksum = checksum ^ (unsigned int)msgSync[x];  // Generate our checksum for the sync msg

		 }
		 msgSync[MSG_SIZE_SYNC - 1] = checksum & 0xFF;  // Store the checksum
		 Serial.write(msgSync, MSG_SIZE_SYNC);     // Send the sync ACK
	         ppmState = ppmLOW;                      // Turn ppm LOW (since the signal is probably off)
	         currentChannel = SERVO_COUNT;           // Set our currentChannel to the last channel
	         statusLEDInterval = STATUS_INTERVAL_OK; // Set our status LED interval to OK
	         delay(20);                              // 20ms for client to receive ACK before flushing buffer
	         Serial.flush();                         // Flush the serial down the toilets

	} else if(msgType == MSG_TYPE_CTRL) { // Handle updating the controls

		/* MSG structure - [BEGIN_MSG] [MSG_TYPE] [SERVOS] [BUTTONS] [CHECKSUM]
	         * BEGIN_MSG - 1 byte  - 1 byte msg marker
		 * MSG_TYPE  - 1 byte  - 1 byte msg type marker
	         * SERVOS    - 8 bytes - 1 byte per servo
	         * BUTTONS   - 3 bytes - 2 bits per pin (allow more than on/off, e.g. 3-pos switch)
	         * CHECKSUM  - 1 byte  - 1 byte XOR checksum
	         */
          
	        for(x = 0 ; x < SERVO_COUNT ; x++) {

	          servos[x] = inMsg[x + 1]; // Set latest servo values from msg

	        }

	        for(x = 0 ; x < 3 ; x++) {  // This loop handles 4 buttons at once since each uses 2 bits and we read in 1 byte (2 bits * 4 = 8 bits = 1 byte)

	          buttons[(x * 4)] = (inMsg[x + 9] & B11000000) >> 6;     // Bitwise and against our byte to strip away other button values, then bitshift to 0th and 1st positions
	          buttons[(x * 4) + 1] = (inMsg[x + 9] & B00110000) >> 4; // Same, you can see the bitmask shift to the right as we work out way down the byte
		  buttons[(x * 4) + 2] = (inMsg[x + 9] & B00001100) >> 2; // Same
	          buttons[(x * 4) + 3] = (inMsg[x + 9] & B00000011);      // No bitshift here since our bits are already in 0th and 1st pos.

	        }

	} else if(msgType == MSG_TYPE_PPZ) { // Handle PPZ message
	} else if(msgType == MSG_TYPE_CFG) { // Handle configuration message
	}

}

/* ISR - TIMER1_COMPAT_Vect, generates the PPM signal */

ISR(TIMER1_COMPA_vect) {

  if(ppmState == ppmLOW) {

    OCR1A = PPM_HIGH_PULSE;      // Pin will stay high for PPM_HIGH_PULSE duration
    digitalWrite(PPM_PIN, HIGH); // Pin was low, set it to HIGH
    ppmState = ppmHIGH;          // Set ppmState HIGH
     
  } else if(ppmState == ppmHIGH) {

    if(currentChannel > (SERVO_COUNT - 1)) {
      
      currentChannel = 0;     // Hit max servo count, reset to 0 channel
      OCR1A = PPM_SYNC_PULSE; // This is the sync pulse, set time to sync pulse time
      
    } else {
      
      OCR1A = channels[currentChannel]; // Pin will stay low for channels[currentChannel] duration
      currentChannel++;
    
    }
    digitalWrite(PPM_PIN, LOW); // Pin was high, set it to LOW
    ppmState = ppmLOW;          // Set ppmState LOW

  } else {

    OCR1A = PPM_HIGH_PULSE;     // Short interval (200us)
    digitalWrite(PPM_PIN, LOW); // Pin is set to low (off), PPM is disabled

  }
 
}

/* ISR - TIMER2_COMPA_vect, updates PPM signal array based on servos at 100Hz */

ISR(TIMER2_COMPA_vect) {

  sei(); // Allow this bullshit to be interrupted
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

}
