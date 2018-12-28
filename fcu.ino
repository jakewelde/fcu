#include <SparkFunMPU9250-DMP.h>
#include <BasicLinearAlgebra.h>
#include <MadgwickAHRS.h>

#include "config.h"

// Flash storage (for nv storage on ATSAMD21)
#include <FlashStorage.h>

using namespace BLA;


MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

Madgwick filter;

uint32_t lastBlink = 0;
void blinkLED()
{
  static bool ledState = false;
  digitalWrite(HW_LED_PIN, ledState);
  ledState = !ledState;
}

// to be used for gains and trimming
// FlashStorage(flashLogRate, unsigned short);

Matrix<4,4> W;

void setup()
{

  // Initialize LED, interrupt input, and serial port.
  initHardware();

  // invert thrust matrix
  computeThrustMixer();

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() )
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ;
  }

  filter.begin(CONTROL_FREQ);

}

Matrix<3,3> J = {
  JXX,   0,   0,
    0, JYY,   0,
    0,   0, JZZ
};

// m x'' + kOm x' + kR x = 0;
Matrix<1,1> kR = {K};
Matrix<1,1> kOm = {sqrt(4*MASS*K)};

Matrix<3,1> unhat(Matrix<3,3> M) {
  Matrix<3,1> v = {
    -M(1,2), M(0,2), -M(0,1)
  };
  return v;
}

Matrix<3,3> hat(Matrix<3,1> v) {
  Matrix<3,3> M = {
    0,    -v(2),  v(1),
    v(2),     0, -v(0),
    -v(1), v(0),     0
  };
  return M;
}

float norm(Matrix<3,1> v) {
  return sqrt(v(0) * v(0) + v(1) * v(1) + v(2) * v(2));
}



void loop()
{

  // control loop frequency regulator
  static uint32_t lastUpdate = 0;
  uint32_t now = micros();
  if (now - lastUpdate < CONTROL_DT) {
    return;
  }
  lastUpdate = now;

  // LED heartbeat
  if(now - lastBlink > BLINK_RATE) {
    blinkLED();
    lastBlink = now;
  }

  if ( !imu.fifoAvailable() )
    return; // no sensor data available

  // read new data from buffer
  if ( imu.dmpUpdateFifo() != INV_SUCCESS )
    return; // If that fails (uh, oh), return to top

  // convert sensor readings to SI units
  float ax = imu.calcAccel(imu.ax);
  float ay = imu.calcAccel(imu.ay);
  float az = imu.calcAccel(imu.az);
  float gx = imu.calcGyro(imu.gx) * 0.0174533f;
  float gy = imu.calcGyro(imu.gy) * 0.0174533f;
  float gz = imu.calcGyro(imu.gz) * 0.0174533f;

  // estimate orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  Matrix<3,1> Om = {gx, gy, gz};

  float qw = filter.qw();
  float qx = filter.qx();
  float qy = filter.qy();
  float qz = filter.qz();

  Matrix<3,3> R = {
    1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw,
    2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw,
    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy
  };

  // TODO : get desired orientations from RX/TX
  Matrix<3,3> Rc = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
  Matrix<3,1> Omc = {0,0,0};
  Matrix<3,1> Omc_dot = {0,0,0};
  double f = 9.81*MASS;
  //  TODO: trimming

  // Compute errors
  Matrix<3,1> eR = unhat((~Rc)*R - (~R)*Rc)*Matrix<1,1>(.5);
  Matrix<3,1> eOm = Om - (~R)*Rc*Omc;

  // Compute control inputs
  Matrix<3,1> M = - eR*kR - eOm*kOm + hat(Om)*J*Om - J*(hat(Om)*(~R)*Rc*Omc - (~R)*Rc*Omc_dot);

  Matrix<4,1> u = {f,M(0),M(1),M(2)};
  Matrix<4,1> wsq = W*u;
  Matrix<4,1> w;
  for(int i = 0; i < 4; i++) {
    if(wsq(i) > 0) {
      w(i) = sqrt(wsq(i));
    } else {
      w(i) = 0;
    }
  }

  // TODO : voltage compensation and monitoring

  // TODO : motor control

  // Print estimation and control data
  // LOG_PORT.print("Orientation:\t");
  // for(int r = 0; r < 3; r++) {
  //   for(int c = 0; c < 3; c++) {
  //     LOG_PORT.print(R(r,c));
  //     LOG_PORT.print("\t");
  //   }
  // }
  // for(int i = 0; i < 3; i++) {
  //   LOG_PORT.print(Om(i));
  //   LOG_PORT.print("\t");
  // }
  //
  // LOG_PORT << M(0) << '\t' << M(1) << '\t' << M(2) << '\t';
  LOG_PORT << u(0) << '\t' << u(1) << '\t' << u(2) << '\t'  << u(3) << '\t';
  LOG_PORT << wsq(0) << '\t' << wsq(1) << '\t' << wsq(2) << '\t'  << wsq(3) << '\t';
  LOG_PORT << w(0) << '\t' << w(1) << '\t' << w(2) << '\t'  << w(3) << '\t';
  LOG_PORT.println();

  // LOG_PORT << W << "\n";

}

void initHardware(void)
{
  // Set up LED pin (active-high, default to off)
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);

  // Set up MPU-9250 interrupt input (active-low)
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);

  // Set up serial log port
  LOG_PORT.begin(SERIAL_BAUD_RATE);
}

bool initIMU(void)
{

  // initialize I2C bus, and reset MPU-9250 to defaults.
  if (imu.begin() != INV_SUCCESS)
    return false;

  // Set up MPU-9250 interrupt:
  imu.enableInterrupt(); // Enable interrupt output
  imu.setIntLevel(1);    // Set interrupt to active-low
  imu.setIntLatched(1);  // Latch interrupt output

  // Configure sensors:
  imu.setGyroFSR(IMU_GYRO_FSR);
  imu.setAccelFSR(IMU_ACCEL_FSR);
  imu.setLPF(IMU_AG_LPF);

  // (note: this value will be overridden by the DMP sample rate)
  imu.setSampleRate(IMU_AG_SAMPLE_RATE); // accel and gyro rate

  // Configure digital motion processor
  unsigned short dmpFeatureMask = 0;
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;

  imu.dmpBegin(dmpFeatureMask, DMP_SAMPLE_RATE);

  return true; // Return success

}

void computeThrustMixer(){
  Matrix<4,4> V = {
       KF,   KF,   KF,    KF,
        0, L*KF,    0, -L*KF,
    -L*KF,    0, L*KF,     0,
       KM,  -KM,   KM,   -KM
  };
  W = V.Inverse();
}
