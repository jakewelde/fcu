#include <SparkFunMPU9250-DMP.h>

#include "config.h"

// Flash storage (for nv storage on ATSAMD21)
#include <FlashStorage.h>

MPU9250_DMP imu; // Create an instance of the MPU9250_DMP class

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

}

void loop()
{

  // 1kHz loop frequency regulator
  static uint32_t lastUpdate = 0;
  uint32_t now = millis();
  if (now <= lastUpdate) {
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

  printData();

}

void printData(void)
{
  String imuLog = ""; // Create a fresh line to log

  // imuLog += String(imu.calcAccel(imu.ax)) + ", ";
  // imuLog += String(imu.calcAccel(imu.ay)) + ", ";
  // imuLog += String(imu.calcAccel(imu.az)) + ", ";
  //
  // imuLog += String(imu.calcGyro(imu.gx)) + ", ";
  // imuLog += String(imu.calcGyro(imu.gy)) + ", ";
  // imuLog += String(imu.calcGyro(imu.gz)) + ", ";
  //
  // imuLog += String(imu.calcMag(imu.mx)) + ", ";
  // imuLog += String(imu.calcMag(imu.my)) + ", ";
  // imuLog += String(imu.calcMag(imu.mz)) + ", ";
  //
  imuLog += String(imu.calcQuat(imu.qw), 4) + ", ";
  imuLog += String(imu.calcQuat(imu.qx), 4) + ", ";
  imuLog += String(imu.calcQuat(imu.qy), 4) + ", ";
  imuLog += String(imu.calcQuat(imu.qz), 4) + ", ";
  //
  // imu.computeEulerAngles();
  // imuLog += String(imu.pitch, 2) + ", ";
  // imuLog += String(imu.roll, 2) + ", ";
  // imuLog += String(imu.yaw, 2) + ", ";

  // Remove last comma/space:
  imuLog.remove(imuLog.length() - 2, 2);
  imuLog += "\r\n"; // Add a new line

  LOG_PORT.print(imuLog); // Print log line to serial port

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
  // Set compass sample rate: between 4-100Hz

  imu.setCompassSampleRate(IMU_COMPASS_SAMPLE_RATE);

  // Configure digital motion processor
  unsigned short dmpFeatureMask = 0;
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_GYRO;
  dmpFeatureMask |= DMP_FEATURE_SEND_RAW_ACCEL;
  dmpFeatureMask |= DMP_FEATURE_6X_LP_QUAT;

  imu.dmpBegin(dmpFeatureMask, DMP_SAMPLE_RATE);

  return true; // Return success

}
