#define SERVO_COUNT 8
#define SIGNAL_PIN 13
#define THROTTLE_SERVO 1

int servoValues[SERVO_COUNT];
int servoPins[SERVO_COUNT] = {2, 3, 4, 5, 6, 7, 9};
int currentPulse;
int currentPulseOut;

long pulseBeginTime;
long lightCounter;

boolean pulse = false;
boolean servoPulse = false;

boolean light = true;

void setup() {

	Serial.begin(115200);
	Serial.println("PPM Decoder Test, debug output.");

        pinMode(SIGNAL_PIN, OUTPUT);

	lightCounter = 1;
	currentPulse = 0;
	currentPulseOut = 0;

        int x, y;
        
	Serial.print("Servo init..");       

	for(x = 0 ; x < SERVO_COUNT ; x++) {

		pinMode(servoPins[x], OUTPUT);

	}

       	for(x = 0 ; x < SERVO_COUNT ; x++) {
          
                  if(x != THROTTLE_SERVO) {
                    
			servoValues[x] = 3000;			
        
                  } else {

			servoValues[x] = 2000;
                    
                  }
          
        }

	// Set up timer1 - ICP1 input pulse

	TIMSK1 = B00100010; // Input capture interrupt enabled, OCR1A = TCNT1 interrupt enabled
	TCCR1A = B00000000; // Normal mode
	TCCR1B = B11000010; // Clock prescaler / 8, rising edge in ICP1 = interrupt

	/*// Set up timer2 - Servo output control

	TIMSK2 = B00000010; // Interrupt on OCR2A = TCNT2
	TCCR2A = B00000010; // CTC mode
	TCCR2B = B00000101; // CTC mode, /128 prescaler, 8 usec interval
	OCR2A = 250; // execute after 500usec*/

	Serial.println(".done.. 5s delay.");
	for(x = 0 ; x < 10 ; x++) {

		digitalWrite(SIGNAL_PIN, light);
		light = !light;

	       	for(y = 0 ; y < SERVO_COUNT ; y++) { // wiggle them servos
          
			if(y != THROTTLE_SERVO) {
                    
				if(x % 2 == 0) {

					servoValues[y] = 2800;			

				} else {

					servoValues[y] = 3200;			

				}
	        
	        	}
	          
       		}
		delay(500);

	}
	digitalWrite(SIGNAL_PIN, HIGH);

}
void loop() {

	if(lightCounter % 8 == 0) {

		digitalWrite(SIGNAL_PIN, light);
		light = !light;
		lightCounter = 0;

	} 

}

ISR(TIMER1_CAPT_vect) {

	long _ICR1 = ICR1;

	if(pulse) {

		pulseBeginTime = _ICR1;
		if(currentPulse == SERVO_COUNT) {

			currentPulse = 0;

		}

		TCCR1B = B10000010; // Clock prescaler / 8, rising edge on ICP1 = interrupt

	} else {

		long timeDiff;
		if(_ICR1 > pulseBeginTime) {

			timeDiff = _ICR1 - pulseBeginTime;

		} else {

			timeDiff = _ICR1 + (65535 - pulseBeginTime);

		}

		if(timeDiff > 6000) {
		
			currentPulse = 0;
		
		} else {

			if(timeDiff > 1900) {

				if(abs(servoValues[currentPulse] - timeDiff) > 8) {

					servoValues[currentPulse] = timeDiff;

				}

				currentPulse++;
	
			}


		}

		TCCR1B = B11000010; // Clock prescaler / 8, falling edge on ICP1 = interrupt

	}

	pulse = !pulse;
	lightCounter++;

}

ISR(TIMER1_COMPA_vect) {

	if(servoPulse) { // We've hit OCR2A in servoPulse mode (8 usec interval, TOP = turn off pulse)

		digitalWrite(servoPins[currentPulseOut], LOW); // Turn off previous servo pin
		currentPulseOut++;
		
		if(currentPulseOut < SERVO_COUNT) {

			OCR1A = TCNT1 + servoValues[currentPulseOut];
			digitalWrite(servoPins[currentPulseOut], HIGH);

		} else { // Go to sleep.. mode

			OCR1A = TCNT1 + 4000;
			servoPulse = false;

		}

	} else {

		servoPulse = true; // Set servoPulse = true so we execute the watcher code next time

		currentPulseOut = 0;
		OCR1A = TCNT1 + servoValues[currentPulseOut];
		digitalWrite(servoPins[currentPulseOut], HIGH); // Turn on first servo pin

	}

}
