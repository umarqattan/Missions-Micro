
// MARK: - Includes
#include "helpers.h"
#include "sensor.h"
#include "mpu.h"
#include "haptics.h"
#include "ble.h"

// MARK: - Defines

#ifdef IS_RIGHT
  #define DEVICE_NAME "RIGHT1_v2"
#endif 

#ifdef IS_LEFT
  #define DEVICE_NAME "LEFT1_v2"
#endif

void setupSensorArrays() 
{
  initByteArray(muxTopByteArray, sizeof(uint16_t)*topSensorsCount);
  initByteArray(muxBottomByteArray, sizeof(uint16_t)*bottomSensorsCount);
  initByteArray(gyroByteArray, sizeof(float)*imuCount);
  initByteArray(accByteArray, sizeof(float)*imuCount);
  initByteArray(magByteArray, sizeof(float)*imuCount);
}

void setSensorsCharacteristic(Sensors side, char *values, int length)
{
    if (start)
    {
        switch (side) 
        {
            case top:
                pTopSensorsCharacteristic->setValue((uint8_t *)values, length);
                pTopSensorsCharacteristic->notify();
                break;
            case bottom:
                pBottomSensorsCharacteristic->setValue((uint8_t *)values, length);
                pBottomSensorsCharacteristic->notify();
                break;
        }
    }
}

void sensorMuxLoop()
{
  uint16_t src1, src2;
  for (int i = 0; i < 16; i++)
  {
    muxIntArray[i] = (uint16_t)readMux(i);
  }

  for (int i = 0; i < bottomSensorsCount; i++)
  {
    src1 = muxIntArray[i];
    MEMCPY(&muxBottomByteArray[2*i], &src1, sizeof(src1));
  }

  for (int i = 0; i < topSensorsCount; i++) 
  {
    src2 = muxIntArray[i+bottomSensorsCount];
    MEMCPY(&muxTopByteArray[2*i], &src2, sizeof(src2));
  }

  setSensorsCharacteristic(top, muxTopByteArray, topSensorsCount*sizeof(src1));
  setSensorsCharacteristic(bottom, muxBottomByteArray, bottomSensorsCount*sizeof(src2));
  
  Serial.print("Bottom\n");
  Serial.print("[ ");
  for (int i = 0; i < bottomSensorsCount; i++)
  {
    MEMCPY(&src1, &muxBottomByteArray[2*i], sizeof(src1));
    Serial.printf("%d ", src1);
  }
  Serial.print("]\n");

  Serial.print("Top\n");
  Serial.print("[ ");
  for (int i = 0; i < topSensorsCount; i++)
  {
    MEMCPY(&src2, &muxTopByteArray[2*i], sizeof(src2));
    Serial.printf("%d ", src2);
  }
  Serial.print("]\n");

}

void setIMUCharacteristic(IMUDevice device) 
{
    switch(device)
    {
        float x, y, z;
        case acc:
            x = IMU.getAccelX_mss();
            y = IMU.getAccelY_mss();
            z = IMU.getAccelZ_mss();
            MEMCPY(&accByteArray[0], &x, sizeof(x));
            MEMCPY(&accByteArray[4], &y, sizeof(y));
            MEMCPY(&accByteArray[8], &z, sizeof(z));
            pAccelerometerCharacteristic->setValue((uint8_t *)accByteArray, imuCount*sizeof(x));
            pAccelerometerCharacteristic->notify();
            break;
        case gyro:
            x = IMU.getGyroX_rads();
            y = IMU.getGyroY_rads();
            z = IMU.getGyroZ_rads();
            MEMCPY(&gyroByteArray[0], &x, sizeof(x));
            MEMCPY(&gyroByteArray[4], &y, sizeof(y));
            MEMCPY(&gyroByteArray[8], &z, sizeof(z));
            pGyroscopeCharacteristic->setValue((uint8_t *)gyroByteArray, imuCount*sizeof(x));
            pGyroscopeCharacteristic->notify();
            break;
        case mag:
            x = IMU.getMagX_uT();
            y = IMU.getMagY_uT();
            z = IMU.getMagZ_uT();
            MEMCPY(&magByteArray[0], &x, sizeof(x));
            MEMCPY(&magByteArray[4], &y, sizeof(y));
            MEMCPY(&magByteArray[8], &z, sizeof(z));
            pMagnetometerCharacteristic->setValue((uint8_t *)magByteArray, imuCount*sizeof(x));
            pMagnetometerCharacteristic->notify();
            break;
    }
}

void mpu9250BLELoop()
{

  IMU.readSensor();

  setIMUCharacteristic(acc);
  setIMUCharacteristic(gyro);
  setIMUCharacteristic(mag);

  #ifdef DEBUG_BLE_MPU_9250
    float x, y, z;
    MEMCPY(&x, &accByteArray[0], sizeof(x));
    MEMCPY(&y, &accByteArray[4], sizeof(y));
    MEMCPY(&z, &accByteArray[8], sizeof(z));

    Serial.printf("[Acc]\nx:%f\ny:%f\nz:%f\n\n", x, y, z);

    MEMCPY(&x, &gyroByteArray[0], sizeof(x));
    MEMCPY(&y, &gyroByteArray[4], sizeof(y));
    MEMCPY(&z, &gyroByteArray[8], sizeof(z));

    Serial.printf("[Gyro]\nx:%f\ny:%f\nz:%f\n\n", x, y, z);

    MEMCPY(&x, &magByteArray[0], sizeof(x));
    MEMCPY(&y, &magByteArray[4], sizeof(y));
    MEMCPY(&z, &magByteArray[8], sizeof(z));

    Serial.printf("[Mag]\nx:%f\ny:%f\nz:%f\n\n", x, y, z);
  #endif

}

void setup() 
{
  Serial.begin(115200); 
  v2BLESetup(DEVICE_NAME);
  setupSensorArrays();
  setupMux();
  setupHaptics();
  setupMPU9250();

}

void loop() {
  if (deviceConnected)
  {
    mpu9250BLELoop();
    sensorMuxLoop();
    HapticsState state = fireHapticsEffect(effect);

    if (state == done)
    {
      pHapticsCharacteristic->setValue("1");
      pHapticsCharacteristic->notify();
    } 
    if (state == incomplete)
    {
      pHapticsCharacteristic->setValue("2");
      pHapticsCharacteristic->notify();
    }
  }

  delay(samplingRate);
}


