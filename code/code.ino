#define EI_ARDUINO_INTERRUPTED_PIN
#include "Macros.h"
#include <EnableInterrupt.h>
#include "Motor.h"

#define M1_ENCODER_PIN  2
#define M2_ENCODER_PIN  3
#define M3_ENCODER_PIN  4

#define M1_PWM_PIN      9
#define M1_DIR_PIN      8

#define M2_PWM_PIN      10
#define M2_DIR_PIN      12

#define M3_PWM_PIN      11
#define M3_DIR_PIN      13

int STD_LOOP_TIME = 9;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

Motor m1(M1_ENCODER_PIN, M1_PWM_PIN, M1_DIR_PIN);
Motor m2(M2_ENCODER_PIN, M2_PWM_PIN, M2_DIR_PIN);
Motor m3(M3_ENCODER_PIN, M3_PWM_PIN, M3_DIR_PIN);

/****************************************************
 *  State variables
 */
// displacement of the ball
double xs = 0;
double ys = 0;
// velocity of the ball
double xs_dot = 0;
double ys_dot = 0;

// angular displacement (calculated from Kalman)
double psi_x = 0;
double psi_y = 0;
double psi_z = 0;
// angular velocity (measured from gyro)
double psi_dot_x = 0;
double psi_dot_y = 0;
double psi_dot_z = 0;
// angular acceleration (measured from acc)
double psi_acc_x = 0;
double psi_acc_y = 0;
double psi_acc_z = 0;

/********************************
 * LQR Control
 * rows correspond to tau_1, tau_2, tau_3
 * columns correspond to xs, ys, phi_x, phi_y, phi_z, and derivatives
 */
static double K[3][10] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

void setup() {
  setupIMU();
  setupMotors();

}

void setupIMU() {

}

void setupMotors() {
  m1.setup();
  m2.setup();
  m3.setup();
  enableInterrupt(m1.encoderPin, Increment, FALLING);
  enableInterrupt(m2.encoderPin, Increment, FALLING);
  enableInterrupt(m3.encoderPin, Increment, FALLING);
}

void Increment() {
  //  analogWrite(5, m1Encoder);
  int pin = arduinoInterruptedPin;
  if (pin == m1.encoderPin) {
    m1.encoder++;
  } else if (pin == m2.encoderPin) {
    m2.encoder++;
  } else if (pin == m3.encoderPin) {
    m3.encoder++;
  } else {
    Sprintln("Increment on unexpected pin");
  }
}

void loop()  {
  sampleIMU();
  sampleEncoders();
  // TODO: calculate ball position
  calculateControl();
  sendControl();
  printInfo();
// *********************** loop timing control **************************
  lastLoopUsefulTime = millis()-loopStartTime;
  if(lastLoopUsefulTime<STD_LOOP_TIME)         delay(STD_LOOP_TIME-lastLoopUsefulTime);
  lastLoopTime = millis() - loopStartTime;
  loopStartTime = millis();
}

void sampleIMU() {

}

void sampleEncoders() {

}

void calculateControl() {

}

void sendControl() {

}

void printInfo() {
  
}