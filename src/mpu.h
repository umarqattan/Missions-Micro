// MARK: - Includes
#include "MPU9250.h"
#include "SparkFunMPU9250-DMP.h"

// MARK: - Defines
#define DEBUG_BLE_MPU_9250
#define ACC_ON 0x1 << 5
#define GYRO_ON 0x1 << 6
#define MAG_ON 0x1 << 7


// MARK: - MPU9250 variables
MPU9250_DMP imu;

uint8_t accByteArray[6];
uint8_t gyroByteArray[6];
uint8_t magByteArray[6]; 
uint8_t imuByteArray[19];
uint8_t gestureConfigValues[2];

void setupMPU9250();

void setIMUData()
{
    int16_t accelX = floatToFixed(imu.calcAccel(imu.ax));
    int16_t accelY = floatToFixed(imu.calcAccel(imu.ay));
    int16_t accelZ = floatToFixed(imu.calcAccel(imu.az));
    int16_t gyroX = floatToFixed(imu.calcGyro(imu.gx));
    int16_t gyroY = floatToFixed(imu.calcGyro(imu.gy));
    int16_t gyroZ = floatToFixed(imu.calcGyro(imu.gz));
    int16_t magX = floatToFixed(imu.calcMag(imu.mx));
    int16_t magY = floatToFixed(imu.calcMag(imu.my));
    int16_t magZ = floatToFixed(imu.calcMag(imu.mz));

    MEMCPY(&accByteArray[0], &accelX, sizeof(accelX));
    MEMCPY(&accByteArray[2], &accelY, sizeof(accelY));
    MEMCPY(&accByteArray[4], &accelZ, sizeof(accelZ));

    MEMCPY(&gyroByteArray[0], &gyroX, sizeof(gyroX));
    MEMCPY(&gyroByteArray[2], &gyroY, sizeof(gyroY));
    MEMCPY(&gyroByteArray[4], &gyroZ, sizeof(gyroZ));

    MEMCPY(&magByteArray[0], &magX, sizeof(magX));
    MEMCPY(&magByteArray[2], &magY, sizeof(magY));
    MEMCPY(&magByteArray[4], &magZ, sizeof(magZ));

    Serial.printf("acc:  [%d, %d, %d]\n", accelX, accelY, accelZ);
    Serial.printf("gyro: [%d, %d, %d]\n", gyroX, gyroY, gyroZ);
    Serial.printf("mag:  [%d, %d, %d]\n", magX, gyroY, gyroZ);
}