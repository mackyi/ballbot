#include "Motor.h"

void Motor::setup() {
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
}

void Motor::move(double tau) {
	// double change = tau*.2/.694;
	// rpm+=change;
	rpm = 0.8*rpm + 0.2*(tau*14);
	if(rpm > 70) {
		rpm = 70;
	}
	if(rpm < -70) {
		rpm = -70;
	}
	double speed = rpm;
	if(speed < 0) {
		speed = -speed;
	}
	analogWrite(pwmPin, speed);
	if(rpm < 0) {
		digitalWrite(dirPin, HIGH);
	} else {
		digitalWrite(dirPin, LOW);
	}
}	