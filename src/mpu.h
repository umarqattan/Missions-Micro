// MARK: - Includes
#include "MPU9250.h"
#include "SparkFunMPU9250-DMP.h"

// MARK: - Defines
#define DEBUG_BLE_MPU_9250
#define ACC_ON 0x1 << 5
#define GYRO_ON 0x1 << 6
#define MAG_ON 0x1 << 7

#define ACC_SIZE 12
#define GYRO_SIZE 12
#define MAG_SIZE 12
#define QUAT_SIZE 16
#define IMU_SIZE 57

// MARK: - MPU9250 variables
MPU9250_DMP imu;

uint8_t accByteArray[ACC_SIZE];
uint8_t gyroByteArray[GYRO_SIZE];
uint8_t magByteArray[MAG_SIZE];
uint8_t quatByteArray[QUAT_SIZE];
uint8_t imuByteArray[IMU_SIZE];

float accelX = 0;
float accelY = 0;
float accelZ = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float magX = 0;
float magY = 0;
float magZ = 0;

void setupMPU9250();

void setIMUData()
{
    accelX = imu.calcAccel(imu.ax);
    accelY = imu.calcAccel(imu.ay);
    accelZ = imu.calcAccel(imu.az);
    gyroX = imu.calcGyro(imu.gx);
    gyroY = imu.calcGyro(imu.gy);
    gyroZ = imu.calcGyro(imu.gz);
    magX = imu.calcMag(imu.mx);
    magY = imu.calcMag(imu.my);
    magZ = imu.calcMag(imu.mz);

    MEMCPY(&accByteArray[0], &accelX, sizeof(accelX));
    MEMCPY(&accByteArray[4], &accelY, sizeof(accelY));
    MEMCPY(&accByteArray[8], &accelZ, sizeof(accelZ));

    MEMCPY(&gyroByteArray[0], &gyroX, sizeof(gyroX));
    MEMCPY(&gyroByteArray[4], &gyroY, sizeof(gyroY));
    MEMCPY(&gyroByteArray[8], &gyroZ, sizeof(gyroZ));

    MEMCPY(&magByteArray[0], &magX, sizeof(magX));
    MEMCPY(&magByteArray[4], &magY, sizeof(magY));
    MEMCPY(&magByteArray[8], &magZ, sizeof(magZ));

    Serial.printf("acc:  [%f, %f, %f]\n", accelX, accelY, accelZ);
    Serial.printf("gyro: [%f, %f, %f]\n", gyroX, gyroY, gyroZ);
    Serial.printf("mag:  [%f, %f, %f]\n", magX, gyroY, gyroZ);
}