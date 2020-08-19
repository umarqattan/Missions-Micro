
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

    initByteArray(muxByteArray, (sensorsCount + 1)*sizeof(uint8_t));
    initByteArray(imuByteArray, 19*sizeof(uint8_t));
    initByteArray(magByteArray, 6*sizeof(uint8_t));
    initByteArray(accByteArray, 6*sizeof(uint8_t));
    initByteArray(gyroByteArray, 6*sizeof(uint8_t));
}

void sensorMuxLoopCode(void *pvParameters)
{
    while(1)
    {
        if (deviceConnected && pressureSensorsConnected)
        {
            muxByteArray[0] = pressureBitConfiguration;
            for (uint8_t i = 0; i < sensorsCount; i++)
            {
                muxByteArray[i+1] = truncate(readMux(i));
            }
           
            // TODO: calculate the current time stamp since the beginning of the connection
            printPressureByteArray(muxByteArray); 
            pDataCharacteristic->setValue((uint8_t *)muxByteArray, (sensorsCount + 1) * sizeof(uint8_t));
            pDataCharacteristic->notify();
        }

        vTaskDelay(pressureSensorSampleRate / portTICK_PERIOD_MS);
    }
}

void sensorMuxLoop()
{
    if (deviceConnected && pressureSensorsConnected)
    {

        uint8_t src1, src2;
        for (uint8_t i = 0; i < bottomSensorsCount + topSensorsCount; i++)
        {
            muxByteArray[i] = (uint8_t)(readMux(i) >> 4);
        }

        for (uint8_t i = 0; i < bottomSensorsCount; i++)
        {
            src1 = muxByteArray[i];
            MEMCPY(&muxBottomByteArray[i], &src1, sizeof(src1));
        }

        for (uint8_t i = 0; i < topSensorsCount; i++) 
        {
            src2 = muxByteArray[i+bottomSensorsCount];
            MEMCPY(&muxTopByteArray[i], &src2, sizeof(src2));
        }

        Serial.print("Bottom\n");
        Serial.print("[ ");
        for (uint8_t i = 0; i < bottomSensorsCount; i++)
        {
            MEMCPY(&src1, &muxBottomByteArray[i], sizeof(src1));
            Serial.printf("%d ", src1);
        }
        Serial.print("]\n");

        Serial.print("Top\n");
        Serial.print("[ ");
        for (uint8_t i = 0; i < topSensorsCount; i++)
        {
            MEMCPY(&src2, &muxTopByteArray[i], sizeof(src2));
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

                uint8_t i = 0;
                uint8_t a_i = 0;
                uint8_t g_i = 0;
                uint8_t m_i = 0;
                bool imuToggle[3] = {false, false, false};

                imuByteArray[0] = (uint8_t)sensorConfiguration;
                uint8_t imuSensorCount = 0;
                if ((sensorConfiguration >> 5) & 0x1)
                {
                    imuSensorCount++;
                    imuToggle[0] = true;
                }
                if ((sensorConfiguration >> 6) & 0x1)
                {
                    imuSensorCount++;
                    imuToggle[1] = true;
                }
                if ((sensorConfiguration >> 7) & 0x1)
                {
                    imuSensorCount++;
                    imuToggle[2] = true;
                }


                Serial.printf("IMU Sensor Count: %d\n", imuSensorCount);
                while(i < (imuSensorCount*6))
                {
                    if (imuToggle[0] && a_i < 3)
                    {                  
                        imuByteArray[i+1] = accByteArray[a_i];
                        a_i++;
                    }
                    else if (imuToggle[1] && g_i < 3)
                    {
                        imuByteArray[i+1] = gyroByteArray[g_i];
                        g_i++;
                    }
                    else if (imuToggle[2] && m_i < 3)
                    {
                        imuByteArray[i+1] = magByteArray[m_i];
                        m_i++;
                    }
                    i++;
                }
                
                // TODO: calculate the current time stamp since the beginning of the connection
                pDataCharacteristic->setValue((uint8_t*)imuByteArray, (imuSensorCount*6+1)*sizeof(uint8_t));
                pDataCharacteristic->notify();
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