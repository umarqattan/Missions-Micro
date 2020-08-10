
// MARK: - Includes
#include "helpers.h"
#include "sensor.h"
#include "ble.h"
#include <pthread.h>

// MARK: - Defines

#ifdef IS_RIGHT
  #define DEVICE_NAME "RIGHT1_v2"
#endif 

#ifdef IS_LEFT
  #define DEVICE_NAME "LEFT1_v2"
#endif

TaskHandle_t sensorMuxTask;
TaskHandle_t imuTask;
TaskHandle_t hapticsTask;

void setupSensorArrays() 
{
    initByteArray(muxTopByteArray, topSensorsCount*sizeof(uint16_t));
    initByteArray(muxBottomByteArray, bottomSensorsCount*sizeof(uint16_t));
    initByteArray(magByteArray, 3*sizeof(float));
    initByteArray(accByteArray, 3*sizeof(float));
    initByteArray(gyroByteArray, 3*sizeof(float));
}

void setSensorsCharacteristic(Sensors side, char *values, int length)
{
    if (start)
    {
        switch (side) 
        {
            case top:
                // pTopSensorsCharacteristic->setValue((uint8_t *)values, length);
                // pTopSensorsCharacteristic->notify();
                break;
            case bottom:
                // pBottomSensorsCharacteristic->setValue((uint8_t *)values, length);
                // pBottomSensorsCharacteristic->notify();
                break;
        }
    }
}

void sensorMuxLoopCode(void *pvParameters)
{
    while(1)
    {
        if (deviceConnected && pressureSensorsConnected)
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
            
            // pTopSensorsCharacteristic->setValue((uint8_t *)muxTopByteArray, topSensorsCount*sizeof(src1));
            // pTopSensorsCharacteristic->notify();

            // pBottomSensorsCharacteristic->setValue((uint8_t *)muxBottomByteArray, bottomSensorsCount*sizeof(src2));
            // pBottomSensorsCharacteristic->notify();
        }

        vTaskDelay(pressureSensorSampleRate / portTICK_PERIOD_MS);
    }
}

void sensorMuxLoop()
{
    if (deviceConnected && pressureSensorsConnected)
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
        
        // pTopSensorsCharacteristic->setValue((uint8_t *)muxTopByteArray, topSensorsCount*sizeof(src1));
        // pTopSensorsCharacteristic->notify();

        // pBottomSensorsCharacteristic->setValue((uint8_t *)muxBottomByteArray, bottomSensorsCount*sizeof(src2));
        // pBottomSensorsCharacteristic->notify();
    }
}

void mpu9250DMPLoopCode(void *pvParameters)
{
    while(1)
    {
        if (deviceConnected && imuConnected)
        {
            if (imu.dataReady())
            {
                imu.update(imuSensors);
                setIMUData();

                // pMagnetometerCharacteristic->setValue((uint8_t*)magByteArray, 3*sizeof(float));
                // pMagnetometerCharacteristic->notify();

                // pAccelerometerCharacteristic->setValue((uint8_t*)accByteArray, 3*sizeof(float));
                // pAccelerometerCharacteristic->notify();
                
                // pGyroscopeCharacteristic->setValue((uint8_t*)gyroByteArray, 3*sizeof(float));
                // pGyroscopeCharacteristic->notify();

            }
        }
        vTaskDelay(accelGyroSampleRate / portTICK_PERIOD_MS);
    }
}

void setup() 
{
    Serial.begin(115200); 
    v2BLESetup(DEVICE_NAME);
    setupSensorArrays();
    setupMux();
    setupHaptics();
    
    xTaskCreatePinnedToCore(sensorMuxLoopCode, "sensorMuxLoop", 4096, NULL, 1, &sensorMuxTask, 0);
    xTaskCreatePinnedToCore(mpu9250DMPLoopCode, "imuLoop", 4096, NULL, 1, &imuTask, 0);
}

void loop() {
    vTaskDelay(10);
}