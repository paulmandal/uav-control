#define SERVO_COUNT 8
#define SIGNAL_PIN 13
#define SERVO_MAX_DEG 180
#define THROTTLE_SERVO 1
#define FIRST_SERVO_PIN 2

int servoValues[SERVO_COUNT];
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

        int x;
        
	Serial.print("Servo init..");       

	for(x = 0 ; x < SERVO_COUNT - 2 ; x++) {

		pinMode(x + FIRST_SERVO_PIN, OUTPUT);

	}

       	for(x = 0 ; x < SERVO_COUNT ; x++) {
          
                  if(x != THROTTLE_SERVO) {
                    
			servoValues[x] = 1500;			
        
                  } else {

			servoValues[x] = 1000;
                    
                  }
          
        }

	// Set up timer1 - ICP1 input pulse

	TIMSK1 = B00100000; // Input capture interrupt enabled
	TCCR1A = B00000000; // Normal mode
	TCCR1B = B11000010; // Clock prescaler / 8, rising edge in ICP1 = interrupt

	// Set up timer2 - Servo output control

	TIMSK2 = B00000010; // Interrupt on OCR2A = TCNT2
	TCCR2A = B00000010; // CTC mode
	TCCR2B = B00000101; // CTC mode, /128 prescaler, 8 usec interval
	OCR2A = 250; // execute after 500usec

	Serial.println(".done.. 5s delay.");
	for(x = 0 ; x < 10 ; x++) {

		digitalWrite(SIGNAL_PIN, light);
		light = !light;

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

			timeDiff = timeDiff / 2;
			if(timeDiff > 950) {

				if(abs(servoValues[currentPulse] - timeDiff) > 4) {

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

ISR(TIMER2_COMPA_vect) {

	if(servoPulse) { // We've hit OCR2A in servoPulse mode (8 usec interval, TOP = turn off pulse)

		digitalWrite(currentPulseOut + FIRST_SERVO_PIN, LOW); // Turn off previous servo pin
		currentPulseOut++;
		
		if(currentPulseOut < SERVO_COUNT - 2) {

			OCR2A = servoValues[currentPulseOut] / 8;
			digitalWrite(currentPulseOut + FIRST_SERVO_PIN, HIGH);
			TCNT2 = 0;

		} else { // Go to sleep.. mode

			OCR2A = 255;
			TCNT2 = 0;
			servoPulse = false;

		}

	} else {

		servoPulse = true; // Set servoPulse = true so we execute the watcher code next time

		currentPulseOut = 0;
		OCR2A = servoValues[currentPulseOut] / 8; // 1 x 8usec, servoValue is in usec to /8 to get ticks
		digitalWrite(FIRST_SERVO_PIN, HIGH); // Turn on first servo pin
		TCNT2 = 0;

	}

}
