#include <ESP32Servo.h>

Servo myservo;  

int pos=0;
#if defined(ARDUINO_ESP32S2_DEV)
int servoPin = 17;
#else
int servoPin = 18;
#endif
const byte interruptPin =0;

void setup() {
	pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), rotate, CHANGE);
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	myservo.setPeriodHertz(50);    // standard 50 hz servo
	myservo.attach(servoPin, 1000, 2000); // attaches the servo on pin 18 to the servo object
	// using default min/max of 1000us and 2000us
	// different servos may require different min/max settings
	// for an accurate 0 to 180 sweep
}

void loop() {
  myservo.write(0);
  delay(10);
}
void rotate(){
    pos = pos +30;
  if (pos>=180)
    pos=0;
    
}
