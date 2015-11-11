#include "Motor.h"

void Motor::setup() {
  pinMode(encoderPin, INPUT_PULLUP);
  pinMode(dirPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
}