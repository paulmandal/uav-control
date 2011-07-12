/* Structures */

struct messageState {

	unsigned char *messageBuffer;
	int readBytes;
	int length;

};

struct ledBlinker {
  
	unsigned int interval; // interval == 0 is always on, interval < 0 is always off, interval > 0 is an interval
	int pin;
	int state;
	unsigned long lastChanged;
  
};

struct voltageSampler {

	int average;
	unsigned long lastSampleTime;
	int pin;
	int currentSample;
	int *sampleData;

};
