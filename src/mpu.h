// MARK: - Includes
#include "MPU9250.h"

// MARK: - Defines
#define DEBUG_BLE_MPU_9250

// MARK: - MPU9250 variables
MPU9250 IMU(Wire,0x68);
enum IMUDevice { acc, gyro, mag};
int imuCount = 3;
int status;

char accByteArray[12];
char gyroByteArray[12];
char magByteArray[12];
int imuSampleRate = 50;

void setupMPU9250();

void setupMPU9250()
{
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    // start communication with IMU 
    status = IMU.begin();
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      while(1) {}
    }

}

void mpu9250PrintLoop()
{
    // read the sensor
    IMU.readSensor();
    float x, y, z;

    x = IMU.getAccelX_mss();
    y = IMU.getAccelY_mss();
    z = IMU.getAccelZ_mss();
    Serial.printf("[Acc]\nx:%f\ny:%f\nz:%f\n\n", x, y, z);

    x = IMU.getGyroX_rads();
    y = IMU.getGyroY_rads();
    z = IMU.getGyroZ_rads();
    Serial.printf("[Gyro]\nx:%f\ny:%f\nz:%f\n\n", x, y, z);

    x = IMU.getMagX_uT();
    y = IMU.getMagY_uT();
    z = IMU.getMagZ_uT();
    Serial.printf("[Mag]\nx:%f\ny:%f\nz:%f\n\n", x, y, z);

}