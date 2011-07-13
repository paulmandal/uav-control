#define MSG_BUFFER_SIZE     256 // Message buffer size in bytes

#define PPZ_MSG_HEADER_SIZE    3 // PPZ msg header size in bytes
#define VOLTAGE_SAMPLES 5 
#define ADC_SAMPLE_RATE 500
#define VOLTAGE_MEASUREMENTS   2 // Voltage measurement count

#define FLASHING_LIGHTS 1

#define STATUS_INTERVAL 1000

#include "gcs_structs.h"

/* Important Numbers */

typedef enum _messageTypes {

	MTYPE_PING, 
	MTYPE_PING_REPLY, 
	MTYPE_HEARTBEAT, 
	MTYPE_FULL_UPDATE, 
	MTYPE_SINGLE_SERVO, 
	MTYPE_VAR_SERVOS, 
	MTYPE_ALL_SERVOS, 
	MTYPE_BUTTON_UPDATE, 
	MTYPE_PPZ, 
	MTYPE_DEBUG, 
	MTYPE_RESET, 
	MTYPE_STATUS, 
	MTYPE_GCS_STATUS,
	MTYPE_CONFIG,
	MTYPE_REQ_CFG,
	MTYPE_BEGIN

} messageTypes;

typedef enum _voltageSamples {

	COMM = 0,
	VIDEO

} voltageSamples;

typedef enum _lightPins {

	STATUS_LIGHT = 0

} lightPins;

ledBlinker lights[FLASHING_LIGHTS];  // blinking lights state

voltageSampler voltageInfo[VOLTAGE_MEASUREMENTS]; // voltage sample info

int messageSizes[] = {4, 4, 3, 22, 5, -1, 19, 6, -1, -1, -1, 11, 9, -1, 4, -1};

boolean pulse = false;
boolean output = false;
unsigned long lastStatusSent = 0;

messageState gcsMsg;
messageState uavMsg;
rssiState rssiInfo;

void setup() {
  
	Serial.begin(115200);
	Serial.flush();
	Serial1.begin(115200);
	Serial1.flush();

	initMessage(&gcsMsg);
	initMessage(&uavMsg);
	initVoltageMeasurement();

	initRSSI();
	initLights();
	initOutputs();
	initRSSITimer();

}


void loop() {

	unsigned long currentTime = millis();

	if((currentTime - lastStatusSent) > STATUS_INTERVAL) {

		sendStatus();
		lastStatusSent = currentTime;

	}

	checkuavMessages(&uavMsg);
	checkgcsMessages(&gcsMsg);

	getSamples();

	updateLights();
  
}

/* Initialise RSSI struct */

void initRSSI() {

	rssiInfo.rssiCount = 0;
	rssiInfo._totalHigh = 0;
	rssiInfo.totalHigh = 0;
	rssiInfo.pin = 9;

}

/* Initialise timer */

void initRSSITimer() {

	TCCR2A = B00000000; // Normal mode
	TIMSK2 = B00000010;
	OCR2A = 40;
	TCNT2 = 0;
	TCCR2B = B00000010; // 8 prescaler, starts timer
	
}

/* Initialise voltage measurement items */

void initVoltageMeasurement() {

	int x;
	
	for(x = 0 ; x < VOLTAGE_MEASUREMENTS ; x++) {

		voltageInfo[x].average = random(0, 1023);
		voltageInfo[x].lastSampleTime = 0;
		voltageInfo[x].currentSample = 0;

	}

	voltageInfo[COMM].pin = COMM + 1;
	voltageInfo[COMM].sampleData = (int*)calloc(VOLTAGE_SAMPLES, sizeof(int));

	voltageInfo[VIDEO].pin = VIDEO + 1; // dirty, change after new board design
	voltageInfo[VIDEO].sampleData = (int*)calloc(VOLTAGE_SAMPLES, sizeof(int));

}

/* getSamples() - Gather battery samples and/or compute their average voltages */

void getSamples() {

	int x;
	unsigned long currentTime = millis();

	for(x = 0 ; x < VOLTAGE_MEASUREMENTS ; x++) {

		if(voltageInfo[x].currentSample > VOLTAGE_SAMPLES) {

			int y;
			long _average = 0;

			for(y = 0 ; y < VOLTAGE_SAMPLES ; y++) {

				_average = _average + voltageInfo[x].sampleData[y];


			}

			_average = _average / VOLTAGE_SAMPLES;
			voltageInfo[x].average = _average;
			voltageInfo[x].currentSample = 0;

		} else {

			if(currentTime - voltageInfo[x].lastSampleTime > ADC_SAMPLE_RATE) {

				voltageInfo[x].sampleData[voltageInfo[x].currentSample] = analogRead(voltageInfo[x].pin);
				voltageInfo[x].lastSampleTime = currentTime;
				voltageInfo[x].currentSample++;

			}

		}

	}

}

/* initLights - Set up lights */

void initLights() {

	int x;        

        for(x = 0 ; x < FLASHING_LIGHTS ; x++) {
  
		lights[x].interval = -1; // Turn light off
		lights[x].pin = -1;  // Disable light pin
		lights[x].lastChanged = 0;
		lights[x].state = false;

        }

	lights[STATUS_LIGHT].interval = 500;
	lights[STATUS_LIGHT].pin = 8;
	
}

/* initOutputs() - Set output pins up */

void initOutputs() {

  	int x;

	for(x = 0 ; x < FLASHING_LIGHTS ; x++) {

		pinMode(lights[x].pin, OUTPUT);

	}
 
}


/* initMessage() - Initialise message */

boolean initMessage(messageState *msg) {

        int x;
	msg->readBytes = 0;
	msg->length = -1; // Init message.length as header length size
	if((msg->messageBuffer = (unsigned char*)calloc(MSG_BUFFER_SIZE, sizeof(char))) != NULL) {

		return true; // calloc() worked
	
	} else {
	
		return false; // calloc() failed
	
	} 
	
}

/* checkuavMessages() - Check for and handle any incoming messages */

boolean checkuavMessages(messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(msg->length == -1) { // Message has < 0 length, check if anything in messageBuffer can fill that in

		msg->length = getMessageLength(msg);

	}

	if(msg->readBytes < msg->length || msg->length == -1) {  // We either aren't done reading the message or we don't have MTYPE_BEGIN and/or MTYPE and/or PARAM to tell us the real length

		if(Serial1.available() > 0)  {  // Byte is availabe, read it

			testByte = Serial1.read();

			if(msg->readBytes == 0) { // Haven't got MTYPE_BEGIN yet, look for it

				if(testByte == MTYPE_BEGIN) { // Beginning of a messge

					msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
					msg->readBytes++;			       // Increment readBytes

				}

			} else {

				msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
				msg->readBytes++;			       // Increment readBytes

			}

			return true;

		} else {

 			return false;

		}	

	} else { 

		if(msg->length > 0) { // Message is finished, relay it


			Serial.write(msg->messageBuffer, msg->length);

		}

		int x;

		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

			msg->messageBuffer[x] = '\0';

		}

		msg->readBytes = 0;   // Zero out readBytes
		msg->length = -1;     // Set message length to -1
		
		return true;

	}

}

/* checkgcsMessages() - Check for and handle any incoming messages */

boolean checkgcsMessages(messageState *msg) {

	unsigned char testByte = 0x00;
	
	if(msg->length == -1) { // Message has < 0 length, check if anything in messageBuffer can fill that in

		msg->length = getMessageLength(msg);

	}

	if(msg->readBytes < msg->length || msg->length == -1) {  // We either aren't done reading the message or we don't have MTYPE_BEGIN and/or MTYPE and/or PARAM to tell us the real length

		if(Serial.available() > 0)  {  // Byte is availabe, read it

			testByte = Serial.read();

			if(msg->readBytes == 0) { // Haven't got MTYPE_BEGIN yet, look for it

				if(testByte == MTYPE_BEGIN) { // Beginning of a messge

					msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
					msg->readBytes++;			       // Increment readBytes

				}

			} else {

				msg->messageBuffer[msg->readBytes] = testByte; // Add the new byte to our message buffer
				msg->readBytes++;			       // Increment readBytes

			}

			return true;

		} else {

 			return false;

		}	

	} else { 

		if(msg->length > 0) { // Message is finished, relay it


			Serial1.write(msg->messageBuffer, msg->length);

		}

		int x;

		for(x = 0 ; x < MSG_BUFFER_SIZE ; x++) {

			msg->messageBuffer[x] = '\0';

		}

		msg->readBytes = 0;   // Zero out readBytes
		msg->length = -1;     // Set message length to -1
		
		return true;

	}

}

/* getMessageLength(msg) */

int getMessageLength(messageState *msg) {

	if(msg->readBytes == 2) { // Do zero-parameter types first, if we can't find one, see if we have enough characters for one of the parametered types
		
		if(msg->messageBuffer[1] < MTYPE_BEGIN) {

			int size = messageSizes[msg->messageBuffer[1]];

			return size; // We got the message size, or it's a parametered type

		} else {

			return -2; // Invalid type

		}

	} else if(msg->readBytes > 2) { // Didn't find any non-parameter message types, let's see if we have a parametered one

		if(msg->messageBuffer[1] == MTYPE_PPZ || msg->messageBuffer[1] == MTYPE_DEBUG) {

			int msgLength = msg->messageBuffer[2];
			if(msgLength < MSG_BUFFER_SIZE) {
	
				return msgLength;  // PPZ & Debug messages have length as param

			} else {

				return -2; // Bogus message

			}

		} else if(msg->messageBuffer[1] == MTYPE_VAR_SERVOS) {
		
			int numServos = msg->messageBuffer[2];

			if(numServos < 10) {  // If we get close to configInfo.servoCount the sent message would be a ALL_SERVOS or FULL_UPDATE

				return 4 + numServos * 2;  // Convert servo count to # of bytes (2 bytes per servo + begin + type + param + check)

			} else {

				return -2; // Bogus message
			}

		} else if(msg->messageBuffer[1] == MTYPE_CONFIG) {

			int msgLength = msg->messageBuffer[2];
			if(msgLength < MSG_BUFFER_SIZE) {

				return msgLength; 

			} else {

				return -2; // We shouldn't get messages larger than we can handle

			}

		} else {

			return -2; // No valid message types to provide length found

		}

	} else {

		return -1; // Haven't read enough bytes yet

	}

}

/* updateLights() - Update lights based on things */

void updateLights() {

	int x;

	for(x = 0 ; x < FLASHING_LIGHTS ; x++) {

		if(lights[x].pin > 0) {

			if(lights[x].interval > 0) {

				unsigned long currentTime = millis(); // get current time
				if(currentTime - lights[x].lastChanged > lights[x].interval) {
                
	                		digitalWrite(lights[x].pin, lights[x].state);
        	        		lights[x].state = !lights[x].state;
        	        		lights[x].lastChanged = currentTime;
              
       	      			}

			} else if(lights[x].interval == 0) {
                            
				digitalWrite(lights[x].pin, HIGH);
              
	     		} else {	
	
				digitalWrite(lights[x].pin, LOW);	

	          	}

		}
          
        }
	
}


/* sendStatus() - Send status */

void sendStatus() {

	unsigned char *status;

	status = (unsigned char*)calloc(messageSizes[MTYPE_GCS_STATUS], sizeof(char));

	status[0] = MTYPE_BEGIN;
	status[1] = MTYPE_GCS_STATUS;
	status[2] = rssiInfo.totalHigh >> 8;   // High byte
	status[3] = rssiInfo.totalHigh & 255;  // Low byte
	status[4] = voltageInfo[COMM].average >> 8;   // High byte
	status[5] = voltageInfo[COMM].average & 255;  // Low byte
	status[6] = voltageInfo[VIDEO].average >> 8;   // High byte
	status[7] = voltageInfo[VIDEO].average & 255; // Low byte
	status[8] = generateChecksum(status, messageSizes[MTYPE_GCS_STATUS] - 1); // Store our checksum as the last byte
	
	Serial.write(status, messageSizes[MTYPE_GCS_STATUS]);     // Send the status
	lastStatusSent = millis();
	free(status);

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

ISR(TIMER2_COMPA_vect) {

	if(digitalRead(rssiInfo.pin)) {

		rssiInfo._totalHigh++;

	}

	if(rssiInfo.rssiCount == 24000) {

		rssiInfo.totalHigh = rssiInfo._totalHigh / 10;
		rssiInfo.rssiCount = 0;
		rssiInfo._totalHigh = 0;

	}

	TCNT2 = 0;
	OCR2A = 40;

	rssiInfo.rssiCount++;	
	
}
