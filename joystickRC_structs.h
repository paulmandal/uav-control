/* Structures */

struct messageState {

	unsigned char *messageBuffer;
	byte readBytes;
	byte length;

};

struct ledBlinker {
  
	unsigned int interval; // interval == 0 is always on, interval < 0 is always off, interval > 0 is an interval
	byte pin;
	boolean state;
	unsigned long lastChanged;
  
};

struct configValues {

	byte debugPin;                // Pin for debug LED
	byte statusLEDPin;            // Pin for status LED
	byte navlightPin;             // Pin for navlight LED/LEDs
	byte rssiPin;                 // RSSI input pin
	byte mainBatteryPin;          // Main battery input pin
	byte commBatteryPin;          // Comm battery input pin (this board)
	byte videoBatteryPin;         // Video battery input pin
	int lostMessageThreshold;     // Time in ms without a message before assuming we've lost our signal
	int heartbeatInterval;        // Interval in ms to send our heartbeat
	int pingInterval;             // Interval in ms to send pings
	byte servoCount;              // # of servos on this board
	byte buttonCount;             // # of buttons on this board
	byte *buttonPinMap;	      // Mapping of buttons to output pins
	int ppmMinPulse;              // PPM min pulse length in 1/2 usec (default = 2000)
	int ppmMaxPulse;              // PPM max pulse length in 1/2 usec (default = 4000)
	int ppmHighPulse;             // PPM high pulse duration in 1/2 usec (default = 400)
	byte ppmPulses;               // How many pulses are there (servo_count * 2) + 2
	int ppmSyncPulse;             // PPM sync pulse length (bunch of math determines this)
	int statusIntervalSignalLost; // Interval for status to flash when signal is lost
	int statusIntervalOK;         // Interval for status to flash when everything is OK
	int navlightInterval;         // Interval for navlights to flash

};

struct signalState {

	boolean	handShook;
	boolean firstSignalEstablished;
	unsigned char pingData;
	unsigned long lastMessageTime; // Time of last legit message, -100 initially so the PPM won't turn on until we get a real message
	unsigned long lastMessageSentTime;
	byte ctrlCounter;

};

struct ppmState {

	byte currentPulse; // The pulse being sent
	boolean ppmON;     // Is PPM on
	int *pulses;       // How many PPM pulses are there

};
