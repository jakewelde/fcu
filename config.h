////////////////////////
// Serial Port Config //
////////////////////////
#define LOG_PORT SERIAL_PORT_USBVIRTUAL
#define SERIAL_BAUD_RATE 115200 // Serial port baud

////////////////
// LED Config //
////////////////
#define HW_LED_PIN 13        // LED attached to pin 13
#define BLINK_RATE 250000 // Blink rate when only UART logging


///////////////////////////
// Control and Filtering //
///////////////////////////
#define CONTROL_FREQ 100 // Hz
#define CONTROL_DT 1000000/CONTROL_FREQ  // Control interval

/////////////////////////
// IMU Default Configs //
/////////////////////////
#define DMP_SAMPLE_RATE    100 // Logging/DMP sample rate(4-200 Hz)
#define IMU_COMPASS_SAMPLE_RATE 100 // Compass sample rate (4-100 Hz)
#define IMU_AG_SAMPLE_RATE 100 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)
#define IMU_ACCEL_FSR      2 // Accel full-scale range (2, 4, 8, or 16)
#define IMU_AG_LPF         98 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)

//////////////////////////
// Hardware Definitions //
//////////////////////////
// Danger - don't change unless using a different platform
#define MPU9250_INT_PIN 4
#define SD_CHIP_SELECT_PIN 38
#define MPU9250_INT_ACTIVE LOW
