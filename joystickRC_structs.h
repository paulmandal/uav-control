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
