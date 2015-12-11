#define EI_ARDUINO_INTERRUPTED_PIN
#include <EnableInterrupt.h>



int m1PWMPin = 11;
int m1DirPin = 13;
int m1EncoderPin = 6;
volatile int m1Encoder = 0;

int m2PWMPin = 3;
int m2DirPin = 8;
int m2EncoderPin = 4;
volatile int m2Encoder = 0;

int m3PWMPin = 10;
int m3DirPin = 12;
int m3EncoderPin = 5;
volatile int m3Encoder = 0;


void setup() {
  Serial.begin(115200);
  pinMode(m1Encoder, INPUT_PULLUP);
  pinMode(m2Encoder, INPUT_PULLUP);
  pinMode(m3Encoder, INPUT_PULLUP);
  pinMode(m1DirPin, OUTPUT);
  pinMode(m2DirPin, OUTPUT);
  pinMode(m3DirPin, OUTPUT);
  enableInterrupt(m1EncoderPin, Increment, FALLING);
  enableInterrupt(m2EncoderPin, Increment, FALLING);
  enableInterrupt(m3EncoderPin, Increment, FALLING);
  digitalWrite(m1DirPin, HIGH);
  digitalWrite(m2DirPin, HIGH);
  digitalWrite(m3DirPin, HIGH);
  double speed = 30;
  analogWrite(m1PWMPin, speed);
  analogWrite(m2PWMPin, 0);
  analogWrite(m3PWMPin, 0);
}

void Increment() {
  //  analogWrite(5, m1Encoder);
  int pin = arduinoInterruptedPin;
  if (pin == m1EncoderPin) {
    m1Encoder++;
  }
  else if (pin == m2EncoderPin) {
    m2Encoder++;
  } else {
    m3Encoder++;
  }

}

void loop() {
  if (m1Encoder >= 240) {
   // changeDirection(m1PWMPin, m1DirPin);
    //m1Encoder = 0;
    analogWrite(m1PWMPin, 0);
  }
  if (m2Encoder >= 240) {
   // changeDirection(m2PWMPin, m2DirPin);
    //m2Encoder = 0;
    analogWrite(m2PWMPin, 0);
  }
  if (m3Encoder >= 240) {
  //  changeDirection(m3PWMPin, m3DirPin);
    // m3Encoder = 0;
    analogWrite(m3PWMPin, 0);
  }
  // for(int i=0;i<=150;i+=1){
  // double speed = i;
  // analogWrite(m1PWMPin, speed);
  // analogWrite(m2PWMPin, speed);
  // analogWrite(m3PWMPin, speed); 
  // Serial.println(i);
  // delay(1000);
  
  // }
}

void changeDirection(int pwmPin, int dirPin){
    if (digitalRead(dirPin) == HIGH){
      int i = 0;
      for(i = 36; i<= 0; i-=4){
         analogWrite(pwmPin, i);
         delay(30);
       }
      digitalWrite(dirPin, LOW);
      for(i = 0; i<= 40; i += 4){
         analogWrite(pwmPin, i);
         delay(30);
     }
    }else{
      int i = 0;
      for(i = 36; i <= 0; i -= 4){
         analogWrite(pwmPin, i);
         delay(30);
       }
      digitalWrite(dirPin, HIGH);
      for(i = 0; i<= 40; i+=4){
         analogWrite(pwmPin, i);
         delay(30);
     }       
    }
}


