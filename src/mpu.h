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
#define IMU_SIZE 41

// MARK: - MPU9250 variables
MPU9250_DMP imu;

uint8_t accByteArray[ACC_SIZE];
uint8_t gyroByteArray[GYRO_SIZE];
uint8_t magByteArray[MAG_SIZE]; 
uint8_t imuByteArray[IMU_SIZE];

void setupMPU9250();

void setIMUData()
{
    float accelX = imu.calcAccel(imu.ax);
    float accelY = imu.calcAccel(imu.ay);
    float accelZ = imu.calcAccel(imu.az);
    float gyroX = imu.calcGyro(imu.gx);
    float gyroY = imu.calcGyro(imu.gy);
    float gyroZ = imu.calcGyro(imu.gz);
    float magX = imu.calcMag(imu.mx);
    float magY = imu.calcMag(imu.my);
    float magZ = imu.calcMag(imu.mz);

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