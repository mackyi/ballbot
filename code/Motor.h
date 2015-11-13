#ifndef Motor_h
#define Motor_h
#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>

class Motor {
  private:
    int dirPin;
    int pwmPin;
  public:
    volatile int encoder;
    int encoderPin;
    double rpm = 0;
    
    Motor(
      int encoderPin, int pwmPin, int dirPin
    ): encoderPin(encoderPin), dirPin(dirPin), pwmPin(pwmPin), encoder(0) {}
    
    void setup();

    void move(double tau);
};

#endif
