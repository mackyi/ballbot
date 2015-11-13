#define EI_ARDUINO_INTERRUPTED_PIN
#include "Macros.h"
#include <Wire.h>
#include <EnableInterrupt.h>
#include <FreeSixIMU.h>
#include "Motor.h"

#define M1_ENCODER_PIN  2
#define M2_ENCODER_PIN  3
#define M3_ENCODER_PIN  4

#define M2_PWM_PIN      9
#define M2_DIR_PIN      8

#define M1_PWM_PIN      10
#define M1_DIR_PIN      12

#define M3_PWM_PIN      11
#define M3_DIR_PIN      13

int STD_LOOP_TIME = 20;
int lastLoopTime = STD_LOOP_TIME;
int lastLoopUsefulTime = STD_LOOP_TIME;
unsigned long loopStartTime = 0;

Motor m1(M1_ENCODER_PIN, M1_PWM_PIN, M1_DIR_PIN);
Motor m2(M2_ENCODER_PIN, M2_PWM_PIN, M2_DIR_PIN);
Motor m3(M3_ENCODER_PIN, M3_PWM_PIN, M3_DIR_PIN);

FreeSixIMU sixDOF = FreeSixIMU();


/****************************************************
 *  State variables
 */
// displacement of the ball
double xs = 0;
double ys = 0;
// velocity of the ball
double xs_dot = 0;
double ys_dot = 0;


float angles[3]; // yaw pitch roll 
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
// static double K[3][10] = {
//   {0, 0, -15.7431, 0, 0, 0, 0, -2.0976*2, 0, 0},
//   {0, 0, 7.8716, 13.6339, 0, 0, 0, 2.0976, 3.6331, 0},
//   {0, 0, 7.8716, -13.6339, 0, 0, 0, 2.0976, -3.6331, 0},
// };

static double K[3][10] = {
  {0, 0, -15.7431, 0, 0, 0, 0, -2.0976*2, 0, 0},
  {0, 0, 7.8716, 13.6339, 0, 0, 0, 2.0976, 3.6331, 0},
  {0, 0, 7.8716, -13.6339, 0, 0, 0, 2.0976, -3.6331, 0},
};
/***************************
 * output
 */
double tau_1 = 0;
double tau_2 = 0;
double tau_3 = 0;

void setup() {
  Serial.begin(9600);
  setupIMU();
  setupMotors();

}

void setupIMU() {
  Wire.begin();
  delay(5);
  sixDOF.init(); //begin the IMU
  delay(5);
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
  sixDOF.getYawPitchRoll(angles);
  psi_dot_z = (angles[0]-psi_z)/lastLoopTime*1000;
  psi_dot_y = (angles[1]-psi_y)/lastLoopTime*1000;
  psi_dot_x = (angles[2]-psi_x)/lastLoopTime*1000;
  psi_z = angles[0];
  psi_y = angles[1];
  psi_x = angles[2];
}

void sampleEncoders() {

}

void calculateControl() {
  tau_1 = K[0][2]*psi_x + K[0][3]*psi_y+K[0][7]*psi_dot_x+K[0][8]*psi_dot_y;
  tau_2 = K[1][2]*psi_x + K[1][3]*psi_y+K[1][7]*psi_dot_x+K[1][8]*psi_dot_y;
  tau_3 = K[2][2]*psi_x + K[2][3]*psi_y+K[2][7]*psi_dot_x+K[2][8]*psi_dot_y;
}

void sendControl() {
  m1.move(tau_1);
  m2.move(tau_2);
  m3.move(tau_3); 
}

void printInfo() {
  if(loopStartTime % 1000 < 100) {
  Sprintln("Info:");
  Sprint(psi_x);
  Sprint(" | ");  
  Sprint(psi_y);
  Sprint(" | ");
  Sprintln(psi_z);
  Sprint(psi_dot_x);
  Sprint(" | ");  
  Sprint(psi_dot_y);
  Sprint(" | ");
  Sprintln(psi_dot_z);
  Sprint(tau_1);
  Sprint(" | ");  
  Sprint(tau_2);
  Sprint(" | ");
  Sprintln(tau_3);
}
}