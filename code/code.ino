#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#define EI_ARDUINO_INTERRUPTED_PIN
#include "Macros.h"
#include <EnableInterrupt.h>
#include "Motor.h"

#define M1_ENCODER_PIN  6
#define M2_ENCODER_PIN  4
#define M3_ENCODER_PIN  5

#define M2_PWM_PIN      3
#define M2_DIR_PIN      8

#define M3_PWM_PIN      10
#define M3_DIR_PIN      12

#define M1_PWM_PIN      11
#define M1_DIR_PIN      13

#define READY_PIN       7

// Find velocity by taking the change over the last LM_SIZE readings
#define LM_SIZE 10

MPU6050 imu;
bool dmpReady;
uint16_t imuPacketSize;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];
float yOffset = 0;
float xOffset = 0;
float zOffset = 0;
int STD_LOOP_TIME = 10;
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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
  readIMU();
}

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
  {0      , -1.4697 , -10.8617, 0       , -0.1654, 0, 0, -2.1878, 0, 0},
  {1.2728 , 0.7348  , 5.4308  , 9.4065  , -0.1654, 0, 0, 1.0939, 1.8947, 0},
  {-1.2728, 0.7348  , 5.4308  , -9.4065 , -0.1654, 0, 0, 1.0939, -1.8947, 0},
};

static double M[3][3] = {
  {2*sqrt(2)/3, -sqrt(2)/3, -sqrt(2)/3},
  {0, sqrt(6)/3, -sqrt(6)/3},
  {-sqrt(2)/3, -sqrt(2)/3, -sqrt(2)/3},
};

static double RW = .0508;
static double RS = .1397;


/***************************
 * output
 */
double tau_1 = 0;
double tau_2 = 0;
double tau_3 = 0;
double vtau_1 = 0;
double vtau_2 = 0;
double vtau_3 = 0;

long calibrationStart;
bool calibrating;
void setup() {
  Serial.begin(115200);
  Fastwire::setup(100, false);
  setupIMU();
  setupMotors();
  calibrating = true;
  calibrationStart = millis();
}

void setupIMU() {
  long start = micros();
  pinMode(READY_PIN, OUTPUT);
  digitalWrite(READY_PIN, HIGH);
  imu.initialize();
  devStatus = imu.dmpInitialize();
  imu.setXGyroOffset(210);
  imu.setYGyroOffset(-50);
  imu.setZGyroOffset(88);
  imu.setZAccelOffset(1788);

  if (devStatus == 0) {
    imu.setDMPEnabled(true);
    //    enableInterrupt(2, dmpDataReady,  RISING);
    mpuIntStatus = imu.getIntStatus();
    dmpReady = true;
    imuPacketSize = imu.dmpGetFIFOPacketSize();
    Serial.println(micros()-start);
    Serial.println("DMP success");
  } else {
    Serial.println("DMP failed");
  }

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
    m1.increment();
  } else if (pin == m2.encoderPin) {
    m2.increment();
  } else if (pin == m3.encoderPin) {
    m3.increment();
  } else {
    Sprintln("Increment on unexpected pin");
  }
}

void loop()  {
  readIMU();
  if (!calibrating) {
    calculateVelocities();
    sampleEncoders();
    // TODO: calculate ball position
    calculateControl();
    sendControl();
    //  printInfo();
    // *********************** loop timing control **************************
    lastLoopUsefulTime = millis() - loopStartTime;
    if (lastLoopUsefulTime < STD_LOOP_TIME) {
      delay(STD_LOOP_TIME - lastLoopUsefulTime);
    }

    lastLoopTime = millis() - loopStartTime;
    loopStartTime = millis();
  } else {
    if (!((millis() - calibrationStart) < 30000)) {
      imu.dmpGetQuaternion(&q, fifoBuffer);
      imu.dmpGetGravity(&gravity, &q);
      imu.dmpGetYawPitchRoll(angles, &q, &gravity);
      xOffset = angles[2];
      yOffset = angles[1];
      zOffset = angles[0];
      calibrating = false;
      digitalWrite(READY_PIN, LOW);
      Serial.println("done calibrating");
    }
  }
}

void readIMU() {
  mpuIntStatus = imu.getIntStatus();

  // get current FIFO count
  fifoCount = imu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    imu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < imuPacketSize) fifoCount = imu.getFIFOCount();

    // read a packet from FIFO

    imu.getFIFOBytes(fifoBuffer, imuPacketSize);
    sampleIMU();
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= imuPacketSize;

  }
}

static double IMU_DIFF_THRESHOLD = 0.3;
int consecutive_filtered = 0;

void sampleIMU() {
  // Serial.println("sampleimu");
  imu.dmpGetQuaternion(&q, fifoBuffer);
  imu.dmpGetGravity(&gravity, &q);
  imu.dmpGetYawPitchRoll(angles, &q, &gravity);
  double diff_y = angles[1] - psi_y - yOffset;
  double diff_x = angles[2] - psi_x - xOffset;
  if(abs(diff_y) > IMU_DIFF_THRESHOLD || abs(diff_x) > IMU_DIFF_THRESHOLD) {
    Sprint("filtered: ");
    Sprint(angles[2] - xOffset);
    Sprint(" | ");
    Sprint(angles[1] - yOffset);
    Sprint(" | ");
    Sprintln(angles[0] - zOffset);
    consecutive_filtered++;
    if(consecutive_filtered > 2) {
      consecutive_filtered = 0;
      // imu.resetDMP();
      // reset somehow
    }
    return;
  }
  consecutive_filtered = 0;
  // psi_dot_z = (angles[0] - psi_z - zOffset) / lastLoopTime * 1000;
  // psi_dot_y = (angles[1] - psi_y - yOffset) / lastLoopTime * 1000;
  // psi_dot_x = (angles[2] - psi_x - xOffset) / lastLoopTime * 1000;
  psi_z = angles[0] - zOffset;
  psi_y = angles[1] - yOffset;
  psi_x = angles[2] - xOffset;
  if(!calibrating){
  printAngles();
  }

}

void calculateVelocities() {
  psi_dot_x = calcPsiDotX(psi_x);
  psi_dot_y = calcPsiDotY(psi_y);
  psi_dot_z = calcPsiDotZ(psi_z);
  printVelocities();
}

float calcPsiDotZ(float M) {
  static float LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;

  // keep sum updated to improve speed.
  float diff = M - LM[index];
  LM[index] = M;
  index = (index + 1) % LM_SIZE;

  return diff/(STD_LOOP_TIME*LM_SIZE)*1000;
}

float calcPsiDotX(float M) {
  static float LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;

  // keep sum updated to improve speed.
  float diff = M - LM[index];
  LM[index] = M;
  index = (index + 1) % LM_SIZE;

  return diff/(STD_LOOP_TIME*LM_SIZE)*1000;
}

float calcPsiDotY(float M) {
  static float LM[LM_SIZE];      // LastMeasurements
  static byte index = 0;

  // keep sum updated to improve speed.
  float diff = M - LM[index];
  LM[index] = M;
  index = (index + 1) % LM_SIZE;

  return diff/(STD_LOOP_TIME*LM_SIZE)*1000;
}

void sampleEncoders() {
  // get the encoder data
  double phi_1 = m1.encoder*3/180*M_PI;
  double phi_2 = m2.encoder*3/180*M_PI;
  double phi_3 = m3.encoder*3/180*M_PI;
  xs = (0.0239473*phi_1 - 0.041478*phi_2 + 
     0.0239473*phi_3)*cos(psi_z) + (0.0478947*phi_1 - 
     0.0239473*phi_3)*sin(psi_z);
  ys = (0.0478947*phi_1 - 
     0.0239473*phi_3)*cos(psi_z) + (-0.0239473*phi_1 + 
     0.041478*phi_2 - 0.0239473*phi_3)*sin(psi_z);
  Sprint(m1.encoder);
  Sprint("|");
  Sprint(m2.encoder);
  Sprint("|");
  Sprint(m3.encoder);
  Sprint("|");
  Sprint(xs);
  Sprint("|");
  Sprintln(ys);
  // double[][] RMat = {
  //   {cos(psi_z), sin(psi_z)},
  //   {-sin(psi_z), cos(psi_z)},
  // };

}

void calculateControl() {
  tau_1 = K[0][2] * psi_x + K[0][3] * psi_y 
    + K[0][0]*xs + K[0][1]*ys
    + K[0][4]*psi_z;
  vtau_1=K[0][7]*psi_dot_x+K[0][8]*psi_dot_y;
  tau_2 = K[1][2] * psi_x + K[1][3] * psi_y 
    + K[1][0]*xs + K[1][1]*ys
    + K[1][4]*psi_z;
  vtau_2=K[1][7]*psi_dot_x+K[1][8]*psi_dot_y;
  tau_3 = K[2][2] * psi_x + K[2][3] * psi_y + 
    K[2][0]* xs + K[2][1]*ys
    + K[2][4]*psi_z;;
  vtau_3=K[2][7]*psi_dot_x+K[2][8]*psi_dot_y;

  //  Serial.print("Tau 1:");
  //  Serial.print(tau_1);
  //  Serial.print(" | ");
  //  Serial.print(psi_t1);
  //  Serial.print(" | ");
  //  Serial.print(tau_2);
  //  Serial.print(" | ");
  //  Serial.print(psi_t2);
  //  Serial.print(" | ");
  //  Serial.print(tau_3);
  //  Serial.print(" | ");
  //  Serial.println(psi_t3);


}

void sendControl() {
  m1.move(tau_1, vtau_1);
  m2.move(tau_2, vtau_2);
  m3.move(tau_3, vtau_3);
}

void printVelocities() {
  // Sprint(psi_dot_x);
  // Sprint(" | ");
  // Sprintln(psi_dot_y);
  // Sprint(" | ");
  // Sprintln(psi_dot_z);  
}

void printAngles() {
  // Sprintln("angles:");
  // Sprint(psi_x);
  // Sprint(" | ");
  // Sprint(psi_y);
  // Sprint(" | ");
  // Sprintln(psi_z);
  // //  Sprint(" | ");
  //  Sprint(psi_dot_x);
  //  Sprint(" | ");
  //  Sprint(psi_dot_y);
  //  Sprint(" | ");
  //  Sprintln(psi_dot_z);
}
void printInfo() {
  if (loopStartTime % 1000 < 100) {
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
