/* Structures */

struct messageState {

	unsigned char *messageBuffer;
	int readBytes;
	int length;

};

struct ledBlinker {
  
  unsigned int interval;
  int pin;
  int state;
  int enabled;
  unsigned long lastChanged;
  
};
