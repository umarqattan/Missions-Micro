
// MARK: - Includes
#include "helpers.h"
#include "sensor.h"
#include "ble.h"
#include <pthread.h>
#include <FreeRTOS.h>

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

bool imuHasBegunTimer = false;
uint32_t imuTimeStart = 0;
uint32_t imuCurrentTime = 0;

bool pressureHasBegunTimer = false;
uint32_t pressureTimeStart = 0;
uint32_t pressureCurrentTime = 0;

void setupSensorArrays() 
{
    initByteArray(muxByteArray, 21*sizeof(uint8_t));
    initByteArray(imuByteArray, 23*sizeof(uint8_t));
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

            if (!pressureHasBegunTimer)
            {
                pressureHasBegunTimer = true;
                pressureTimeStart = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                MEMCPY(&muxByteArray[sensorsCount + 1], &pressureCurrentTime, sizeof(uint32_t));
            } 
            else 
            {
                pressureCurrentTime = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - pressureTimeStart;
                MEMCPY(&muxByteArray[sensorsCount + 1], &pressureCurrentTime, sizeof(uint32_t));
            }

            uint8_t byteCount = sensorsCount + 5; // 16 sensors + 1 configuration + 4 time stamp bytes
            pDataCharacteristic->setValue((uint8_t *)muxByteArray, byteCount * sizeof(uint8_t));
            pDataCharacteristic->notify();
            
            printPressureByteArray(muxByteArray); 
        }
        else
        {
            pressureHasBegunTimer = false;
            pressureCurrentTime = 0;
            pressureTimeStart = 0;
        }

        vTaskDelay(pressureSensorSampleRate / portTICK_PERIOD_MS);
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
                while(i < 6 * imuSensorCount) // 3 imu sensors (a, g, m) * 2 bytes per axis * 3 axes per sensor
                {
                    if (imuToggle[0] && a_i < 6)
                    {                  
                        imuByteArray[i+1] = accByteArray[a_i];
                        a_i++;
                    }
                    else if (imuToggle[1] && g_i < 6)
                    {
                        imuByteArray[i+1] = gyroByteArray[g_i];
                        g_i++;
                    }
                    else if (imuToggle[2] && m_i < 6)
                    {
                        imuByteArray[i+1] = magByteArray[m_i];
                        m_i++;
                    }
                    i++;
                }

                if (!imuHasBegunTimer)
                {
                    imuHasBegunTimer = true;
                    imuTimeStart = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                    MEMCPY(&imuByteArray[i], &imuCurrentTime, sizeof(uint32_t));
                } 
                else 
                {
                    imuCurrentTime = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - imuTimeStart;
                    MEMCPY(&imuByteArray[i], &imuCurrentTime, sizeof(uint32_t));
                }

                uint8_t byteCount = 6 * imuSensorCount + 5; // 3 imu sensors (a, g, m) * 2 bytes per axis * 3 axes per sensor + 1 configuration byte + 4 time stamp bytes
                pDataCharacteristic->setValue((uint8_t*)imuByteArray, byteCount * sizeof(uint8_t));
                pDataCharacteristic->notify();

                Serial.printf("[ ");
                for (uint8_t i = 0; i < 23; i++)
                {
                    Serial.printf("%d ", imuByteArray[i]);
                }
                Serial.println("]");

            }
        }
        else
        {
            imuHasBegunTimer = false;
            imuCurrentTime = 0;
            imuTimeStart = 0;
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