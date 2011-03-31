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
#define VERSION_MINOR 0     // Minor #
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
#define STATUS_INTERVAL_SIGNAL_LOST 250 // Toggle every 250ms
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
enum ppmStates ppmState = ppmOFF;           // PPM status
unsigned int channels[SERVO_COUNT];  // Servo channels

/* Function prototypes */

void initControlState();
void initTimer();
void initPPM();
void initOutputs();
void updateStatusLED();
void updateNavigationLights();
void checkMessages();
void checkSignal();
void storePulse(byte targetChannel, int inValue, int inRangeLow, int inRangeHigh);
void readSerialMessage(unsigned char *buffer, int length);

/* Setup function */

void setup() {

  initControlState();   // Initialise control state
  initOutputs();        // Initialise outputs
  initPPM();            // Enable PPM 
  initTimer();          // Turn on Timer
  Serial.begin(38400);  // Open XBee/GCS Serial
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

  int x, msgSize;
  boolean syncMsg = false;
  unsigned int checksum;
  unsigned char inMsg[128]; // Serial buffer is only 128 bytes anyway
  unsigned char testByte;
  unsigned char msgType;

  if(Serial.available() > 1) {         // Need at very least the MSG_BEGIN and MSG_TYPE characters (2 bytes)
 
    testByte = Serial.read();            // Read the first byte in the buffer
    if(testByte == MSG_BEGIN) {

	inMsg[0] = testByte;           // Store the test byte
	msgType = Serial.read();       // Grab the msgType indicator byte
	inMsg[1] = msgType;	       // Store the msgType byte
        if(msgType == MSG_TYPE_SYNC) { // Message is a sync message, handle it that way

		msgSize = MSG_SIZE_SYNC;
		
	} else if(msgType == MSG_TYPE_CTRL) {

		msgSize = MSG_SIZE_CTRL;

	} else if(msgType == MSG_TYPE_PPZ) {

		msgSize = MSG_SIZE_PPZ;

	} else if(msgType == MSG_TYPE_CFG) {
		
		msgSize = MSG_SIZE_CFG;
	
	} else { // Not a valid message type?  Ignore this garbage

		msgSize = 0;

	}

	readSerialMessage(inMsg, msgSize);  // Read the acutal message into our buffer

	checksum = 0x00;

	for(x = 0 ; x < msgSize ; x++) {

        	checksum = checksum ^ (unsigned int)inMsg[x];  // Test the message against its checksum

	}

	if(checksum == 0x00) {  // Checksum was valid, this message should be legitimate

		lostSignal = false;
		lastMsgTime = millis();

		if(msgType == MSG_TYPE_SYNC) {  // Handle the message, since it got past checksum it has to be legit

			 Serial.println("ACK");                  // send ACK for sync msg
		         ppmState = ppmLOW;                      // turn ppm LOW (since the signal is probably off)
		         currentChannel = SERVO_COUNT;           // set our currentChannel to the last channel
		         statusLEDInterval = STATUS_INTERVAL_OK; // set our status LED interval to OK
		         delay(20);                              // 20ms for client to receive ACK before flushing buffer
		         Serial.flush();                         // Flush the serial down the toilets

		} else if(msgType == MSG_TYPE_CTRL) { // Handle updating the controls

			/* MSG structure - [BEGIN_MSG] [MSG_TYPE] [SERVOS] [BUTTONS] [CHECKSUM]
		         * BEGIN_MSG - 1 byte  - 1 byte msg marker
			 * MSG_TYPE  - 1 byte  - 1 byte msg type marker
		         * SERVOS    - 8 bytes - 1 byte per servo
		         * BUTTONS   - 3 bytes - 2 bits per pin (allow more than on/off, e.g. 3-pos switch)
		         * CHECKSUM  - 1 byte  - 1 byte XOR checksum
		         *
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

    }

    // Discard useless byte in testByte

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

/* readSerialMessage(buffer, length) - Read a message from the Serial buffer */

void readSerialMessage(unsigned char *buffer, int length) {

	int x;
	while(Serial.available() < length) {}
	for(x = 2 ; x < length ; x++) {  // Don't overwrite the msgBegin or msgType bytes

		buffer[x] = Serial.read();

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
