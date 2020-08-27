
// MARK: - Includes
#include "helpers.h"
#include "sensor.h"
#include "ble.h"
#include <pthread.h>
#include <FreeRTOS.h>
#include "madgwick.h"

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
    initByteArray(muxByteArray, PRESSURE_SIZE*sizeof(uint8_t));
    initByteArray(imuByteArray, IMU_SIZE*sizeof(uint8_t));
    initByteArray(accByteArray, ACC_SIZE*sizeof(uint8_t));
    initByteArray(gyroByteArray, GYRO_SIZE*sizeof(uint8_t));
    initByteArray(magByteArray, MAG_SIZE*sizeof(uint8_t));
    initByteArray(quatByteArray, QUAT_SIZE*sizeof(uint8_t));
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
                uint16_t value = readMux(i);
                MEMCPY(&muxByteArray[2*i + 1], &value, sizeof(uint16_t));
            }

            if (!pressureHasBegunTimer)
            {
                pressureHasBegunTimer = true;
                pressureTimeStart = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                MEMCPY(&muxByteArray[2*sensorsCount + 1], &pressureCurrentTime, sizeof(uint32_t));
            } 
            else 
            {
                pressureCurrentTime = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - pressureTimeStart;
                MEMCPY(&muxByteArray[2*sensorsCount + 1], &pressureCurrentTime, sizeof(uint32_t));
            }

            uint8_t byteCount = 2*sensorsCount + 5; // 16 sensors + 1 configuration + 4 time stamp bytes
            pDataCharacteristic->setValue((uint8_t *)muxByteArray, byteCount * sizeof(uint8_t));
            pDataCharacteristic->notify();
            
            #ifdef DEBUG_BLE_SENSORS
                printPressureByteArray(muxByteArray); 
            #endif
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

                while(i < ACC_SIZE * imuSensorCount) // 3 imu sensors (a, g, m) * 4 bytes per axis * 3 axes per sensor
                {
                    if (imuToggle[0] && a_i < ACC_SIZE)
                    {                  
                        imuByteArray[i+1] = accByteArray[a_i];
                        a_i++;
                    }
                    else if (imuToggle[1] && g_i < GYRO_SIZE)
                    {
                        imuByteArray[i+1] = gyroByteArray[g_i];
                        g_i++;
                    }
                    else if (imuToggle[2] && m_i < MAG_SIZE)
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
                    MEMCPY(&imuByteArray[i+1], &imuCurrentTime, sizeof(uint32_t));
                } 
                else 
                {
                    imuCurrentTime = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - imuTimeStart;
                    MEMCPY(&imuByteArray[i+1], &imuCurrentTime, sizeof(uint32_t));
                    deltat = imuCurrentTime / 1000.0f;
                    MadgwickQuaternionUpdate(-accelX, accelY, accelZ, gyroX*PI/180.0f, -gyroY*PI/180.0f, -gyroZ*PI/180.0f,  magY,  -magX, magZ);
                }

                uint8_t byteCount = ACC_SIZE * imuSensorCount + 5;
                if (imuSensorCount == 3) // calculate quaternion
                {
                    byteCount = byteCount + QUAT_SIZE;
                    for (uint8_t q_i = 0; q_i < 4; q_i++)
                    {
                        MEMCPY(&imuByteArray[i+1+q_i*4], &q[q_i], sizeof(q[q_i]));
                    }
                }

                pDataCharacteristic->setValue((uint8_t*)imuByteArray, byteCount * sizeof(uint8_t));
                pDataCharacteristic->notify();

                #ifdef DEBUG_BLE_MPU_9250
                    printQuatArray(q, 4);
                    printIMUByteArray(imuByteArray, byteCount);
                #endif
            }
        }
        else
        {
            imuHasBegunTimer = false;
            imuCurrentTime = 0;
            imuTimeStart = 0;
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
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