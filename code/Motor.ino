#include "Motor.h"

void Motor::setup() {
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
}

void Motor::move(double tau) {
	rpm = rpm*.8+tau*(.4)*.2;
	if(rpm > 50) {
		rpm = 50;
	}
	if(rpm < -50) {
		rpm = -50;
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