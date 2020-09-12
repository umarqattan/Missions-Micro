
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
uint32_t Now = 0;
uint32_t lastUpdate = 0; 
uint32_t firstUpdate = 0;

bool pressureHasBegunTimer = false;
uint32_t pressureTimeStart = 0;
uint32_t pressureCurrentTime = 0;

void setupIMUTest();

void setupSensorArrays() 
{
    initByteArray(muxByteArray, (PRESSURE_SIZE+4)*sizeof(uint8_t));
    initByteArray(accByteArray, (ACC_SIZE+4)*sizeof(uint8_t));
    initByteArray(gyroByteArray, (GYRO_SIZE+4)*sizeof(uint8_t));
    initByteArray(quatByteArray, (QUAT_SIZE+4)*sizeof(uint8_t));
}

void sensorMuxLoopCode(void *pvParameters)
{
    while(1)
    {
        if (deviceConnected && pressureOn)
        {
            for (uint8_t i = 0; i < sensorsCount; i++)
            {
                muxByteArray[i] = truncate(readMux(i));
            }

            if (!pressureHasBegunTimer)
            {
                pressureHasBegunTimer = true;
                pressureTimeStart = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            } 
            else 
            {
                pressureCurrentTime = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - pressureTimeStart;
            }

            MEMCPY(&muxByteArray[PRESSURE_SIZE], &pressureCurrentTime, sizeof(uint32_t));
            pPressureDataCharacteristic->setValue((uint8_t *)muxByteArray, (PRESSURE_SIZE) * sizeof(uint8_t));
            pPressureDataCharacteristic->notify();
            
            #ifdef DEBUG_BLE_SENSORS
                // printPressureByteArray(muxByteArray); 
            #endif
        }
        else
        {
            pressureHasBegunTimer = false;
            pressureCurrentTime = 0;
            pressureTimeStart = 0;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void mpu9250DMPLoopCode(void *pvParameters)
{
    while(1)
    {

        if (deviceConnected && imuConnected)
        {

            // Check for new data in the FIFO
            if ( imu.fifoAvailable())
            {
                // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
                if ( imu.dmpUpdateFifo() == INV_SUCCESS)
                {
                    setIMUData();

                    if (!imuHasBegunTimer)
                    {
                        imuHasBegunTimer = true;
                        imuTimeStart = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
                    } 
                    else 
                    {
                        imuCurrentTime = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) - imuTimeStart;
                    }

                    if (accOn)
                    {   
                        
                        MEMCPY(&accByteArray[ACC_SIZE], &imuCurrentTime, sizeof(uint32_t));
                        pAccelerometerDataCharacteristic->setValue((uint8_t *)accByteArray, (ACC_SIZE+4)*sizeof(uint8_t));
                        pAccelerometerDataCharacteristic->notify();
                    }

                    if (gyroOn)
                    {  
                        MEMCPY(&gyroByteArray[GYRO_SIZE], &imuCurrentTime, sizeof(uint32_t));
                        pGyroscopeDataCharacteristic->setValue((uint8_t *)gyroByteArray, (GYRO_SIZE+4)*sizeof(uint8_t));
                        pGyroscopeDataCharacteristic->notify();
                    }

                    if (quatOn)
                    {
                        MEMCPY(&quatByteArray[QUAT_SIZE], &imuCurrentTime, sizeof(uint32_t));
                        pQuaternionDataCharacteristic->setValue((uint8_t *)quatByteArray, (QUAT_SIZE+4)*sizeof(uint8_t));
                        pQuaternionDataCharacteristic->notify();
                    }

                    #ifdef DEBUG_BLE_MPU_9250
                        // Serial.printf("[%f, %f, %f, %f]\n", q0, q1, q2, q3);
                        // printIMUByteArray(imuByteArray, ACC_SIZE*3 + QUAT_SIZE + 4 + 1);

                    #endif
                }
            }
        }
        else
        {
            imuHasBegunTimer = false;
            imuCurrentTime = 0;
            imuTimeStart = 0;
        }
        
        delay(10);
    }
}

void setup() 
{
    Serial.begin(115200); 
    v2BLESetup(DEVICE_NAME);
    setupSensorArrays();
    setupMux();
    setupHaptics();
    
    // xTaskCreatePinnedToCore(sensorMuxLoopCode, "sensorMuxLoop", 4096, NULL, 1, &sensorMuxTask, 0);
    xTaskCreatePinnedToCore(mpu9250DMPLoopCode, "imuLoop", 4096, NULL, 1, &imuTask, 0);
}

void loop() {
}
