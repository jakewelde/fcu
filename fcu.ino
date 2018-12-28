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

// to be used for gains
// FlashStorage(flashLogRate, unsigned short);

void setup()
{

  // Initialize LED, interrupt input, and serial port.
  initHardware();

  // Initialize the MPU-9250. Should return true on success:
  if ( !initIMU() )
  {
    LOG_PORT.println("Error connecting to MPU-9250");
    while (1) ;
  }

  filter.begin(CONTROL_FREQ);

}

float yaw;
float pitch;
float roll;

void loop()
{

  // 1kHz loop frequency regulator
  static uint32_t lastUpdate = 0;
  uint32_t now = micros();
  if (now - lastUpdate < CONTROL_DT) {
    return;
  }
  lastUpdate = now;

  if(now - lastBlink > BLINK_RATE) {
    blinkLED();
    lastBlink = now;
  }

  if ( !imu.fifoAvailable() )
    return; // no sensor data available

  // read new data from buffer
  if ( imu.dmpUpdateFifo() != INV_SUCCESS )
    return; // If that fails (uh, oh), return to top

  float ax = imu.calcAccel(imu.ax);
  float ay = imu.calcAccel(imu.ay);
  float az = imu.calcAccel(imu.az);

  float gx = imu.calcGyro(imu.gx);
  float gy = imu.calcGyro(imu.gy);
  float gz = imu.calcGyro(imu.gz);

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();
  LOG_PORT.print("Orientation: ");
  LOG_PORT.print(yaw);
  LOG_PORT.print(" ");
  LOG_PORT.print(pitch);
  LOG_PORT.print(" ");
  LOG_PORT.println(roll);

}

// Matrix<3,3> rotationFromQuaternion(float qw, float qx, float qy, float qz) {
//   Matrix<3,3> R = {
//     1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw,
//     2*qx*qy + 2*qz*qw,     1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw,
//     2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx*qx - 2*qy*qy
//   };
//   return R;
// }

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
