// MARK: - Includes
#include "MPU9250.h"
#include "SparkFunMPU9250-DMP.h"

// MARK: - Defines
#define DEBUG_BLE_MPU_9250

// MARK: - MPU9250 variables
MPU9250_DMP imu;

char accByteArray[12];
char gyroByteArray[12];
char magByteArray[12]; 
char yprByteArray[12];
char gestureConfigValues[2];
float yaw = 0;
float pitch = 0;
float roll = 0;

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

    pitch = atan2(accelY, (sqrt((accelX * accelX) + (accelZ * accelZ))));
    roll = atan2(-accelX, (sqrt((accelY * accelY) + (accelZ * accelZ))));
    float Yh = (magY * cos(roll)) - (magZ * sin(roll));
    float Xh = (magX * cos(pitch)) + (magY * sin(roll) * sin(pitch)) + (magZ * cos(roll) * sin(pitch));

    yaw = atan2(Yh, Xh);

    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw  = yaw * 180.0 / M_PI;

    MEMCPY(&yprByteArray[0], &yaw, sizeof(yaw));
    MEMCPY(&yprByteArray[4], &pitch, sizeof(pitch));
    MEMCPY(&yprByteArray[8], &roll, sizeof(roll));

    MEMCPY(&accByteArray[0], &accelX, sizeof(accelX));
    MEMCPY(&accByteArray[4], &accelY, sizeof(accelY));
    MEMCPY(&accByteArray[8], &accelZ, sizeof(accelZ));

    MEMCPY(&gyroByteArray[0], &gyroX, sizeof(gyroX));
    MEMCPY(&gyroByteArray[4], &gyroY, sizeof(gyroY));
    MEMCPY(&gyroByteArray[8], &gyroZ, sizeof(gyroZ));

    MEMCPY(&magByteArray[0], &magX, sizeof(magX));
    MEMCPY(&magByteArray[4], &magY, sizeof(magY));
    MEMCPY(&magByteArray[8], &magZ, sizeof(magZ));

    Serial.printf("ypr:  [%f, %f, %f]\n", yaw, pitch, roll);
    Serial.printf("acc:  [%f, %f, %f]\n", accelX, accelY, accelZ);
    Serial.printf("gyro: [%f, %f, %f]\n", gyroX, gyroY, gyroZ);
    Serial.printf("mag:  [%f, %f, %f]\n", magX, gyroY, gyroZ);
}