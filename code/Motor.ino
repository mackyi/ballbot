#include "Motor.h"

void Motor::setup() {
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
}

void Motor::move(double tau) {
	// double change = tau*.2/.694;
	// rpm+=change;
	rpm = tau*24;
	if(rpm > 150) {
		rpm = 150;
	}
	if(rpm < -150) {
		rpm = -150;
	}
	double speed = rpm;
	if(speed < 0) {
		speed = -speed;
	}
	analogWrite(pwmPin, speed+5);
	if(rpm < 0) {
		digitalWrite(dirPin, HIGH);
	} else {
		digitalWrite(dirPin, LOW);
	}
}	

void Motor::increment() {
	if(rpm >= 0) {
		encoder++;
	} else {
		encoder--;
	}
}
