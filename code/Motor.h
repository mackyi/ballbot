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
    
    Motor(
      int encoderPin, int dirPin, int pwmPin
    ): encoderPin(encoderPin), dirPin(dirPin), pwmPin(pwmPin), encoder(0) {}
    
    void setup();
};

#endif
