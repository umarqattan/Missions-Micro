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
// uint8_t linAccByteArray[ACC_SIZE];

float accelX = 0;
float accelY = 0;
float accelZ = 0;
float laccelX = 0;
float laccelY = 0;
float laccelZ = 0;
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
float magX = 0;
float magY = 0;
float magZ = 0;
float q0 = 0;
float q1 = 0;
float q2 = 0;
float q3 = 0;


void setupMPU9250();
void setLinAccel();

void setLinAccel()
{
    float gx = 0;
    float gy = 0;
    float gz = 0;

    gx = 2 * (q1 * q3 - q0 * q2);
    gy = 2 * (q0 * q1 + q2 * q3);
    gz = q0 * q0 - q1 * q1 - q2 * q2 + q3* q3;

    laccelX = accelX - gx;
    laccelY = accelY - gy;
    laccelZ = accelZ - gz;
}
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
    q0 = imu.calcQuat(imu.qw);
    q1 = imu.calcQuat(imu.qx);
    q2 = imu.calcQuat(imu.qy);
    q3 = imu.calcQuat(imu.qz);

    // setLinAccel();

    MEMCPY(&accByteArray[0], &accelX, sizeof(accelX));
    MEMCPY(&accByteArray[4], &accelY, sizeof(accelY));
    MEMCPY(&accByteArray[8], &accelZ, sizeof(accelZ));

    // MEMCPY(&linAccByteArray[0], &laccelX, sizeof(laccelX));
    // MEMCPY(&linAccByteArray[4], &laccelY, sizeof(laccelY));
    // MEMCPY(&linAccByteArray[8], &laccelZ, sizeof(laccelZ));

    MEMCPY(&gyroByteArray[0], &gyroX, sizeof(gyroX));
    MEMCPY(&gyroByteArray[4], &gyroY, sizeof(gyroY));
    MEMCPY(&gyroByteArray[8], &gyroZ, sizeof(gyroZ));

    MEMCPY(&magByteArray[0], &magX, sizeof(magX));
    MEMCPY(&magByteArray[4], &magY, sizeof(magY));
    MEMCPY(&magByteArray[8], &magZ, sizeof(magZ));

    MEMCPY(&quatByteArray[0], &q0, sizeof(q0));
    MEMCPY(&quatByteArray[4], &q1, sizeof(q1));
    MEMCPY(&quatByteArray[8], &q2, sizeof(q2));
    MEMCPY(&quatByteArray[12], &q3, sizeof(q3));

    Serial.printf("acc:  [%f, %f, %f]\n", accelX, accelY, accelZ);
    Serial.printf("linear acc:  [%f, %f, %f]\n", laccelX, laccelY, laccelZ);

    // Serial.printf("gyro: [%f, %f, %f]\n", gyroX, gyroY, gyroZ);
    // Serial.printf("mag:  [%f, %f, %f]\n", magX, gyroY, gyroZ);
    // Serial.printf("[%f, %f, %f, %f]\n", q0, q1, q2, q3);
}