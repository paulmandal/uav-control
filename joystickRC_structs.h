/* Structures */

struct messageState {

	unsigned char *messageBuffer;
	int readBytes;
	int length;

};

struct ledBlinker {
  
	unsigned int interval; // interval == 0 is always on, interval < 0 is always off, interval > 0 is an interval
	int pin;
	boolean state;
	unsigned long lastChanged;
  
};

struct configValues {

	int debugPin;                // Pin for debug LED
	int statusLEDPin;            // Pin for status LED
	int navlightPin;             // Pin for navlight LED/LEDs
	int rssiPin;                 // RSSI input pin
	int mainBatteryPin;          // Main battery input pin
	int commBatteryPin;          // Comm battery input pin (this board)
	int videoBatteryPin;         // Video battery input pin
	int lostMessageThreshold;     // Time in ms without a message before assuming we've lost our signal
	int heartbeatInterval;        // Interval in ms to send our heartbeat
	int pingInterval;             // Interval in ms to send pings
	int servoCount;              // # of servos on this board
	int buttonCount;             // # of buttons on this board
	int *buttonPinMap;	      // Mapping of buttons to output pins
	int ppmMinPulse;              // PPM min pulse length in 1/2 usec (default = 2000)
	int ppmMaxPulse;              // PPM max pulse length in 1/2 usec (default = 4000)
	int ppmHighPulse;             // PPM high pulse duration in 1/2 usec (default = 400)
	int ppmPulses;               // How many pulses are there (servo_count * 2) + 2
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
	int ctrlCounter;

};

struct ppmState {

	int currentPulse; // The pulse being sent
	boolean ppmON;     // Is PPM on
	int *pulses;       // How many PPM pulses are there

};
