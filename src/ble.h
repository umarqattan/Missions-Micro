// MARK: - Includes
#include "HardwareSerial.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "mpu.h"
#include "haptics.h"

// V2
#define SERVICE_UUID_1 "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SERVICE_UUID_2 "7a658cba-0dcd-4d02-bb97-80296cf72dfd"
#define SERVICE_UUID_3 "31408399-730b-4d53-911e-993cd531e96f"

#define PRESSURE_SENSOR_CONFIGURATION_CHARACTERISTIC_UUID "9229de5d-1d33-4b66-8ac7-0e4993f743f8"
#define TOP_SENSORS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BOTTOM_SENSORS_CHARACTERISTIC_UUID "0031a9bf-b569-4f8c-9d18-2256c9561d83"

#define IMU_CONFIGURATION_CHARACTERISTIC_UUID "42ac498a-94da-4c68-b242-bcf8d92c17aa"
#define YPR_CHARACTERISTIC_UUID "c7e6d447-a0a7-4e22-bf14-c0be89905302"
#define ACCELEROMETER_CHARACTERISTIC_UUID "142bc492-b0cf-45ea-a496-338f21f1a815"
#define GYROSCOPE_CHARACTERISTIC_UUID "46382de9-48e8-40b2-94fd-4612af2caa96"
#define MAGNETOMETER_CHARACTERISTIC_UUID "be7db1c9-cddf-41cd-8146-cadebae3b308"

#define HAPTICS_CHARACTERISTIC_UUID "ff227c5f-bfef-4db4-8ce0-e4ef8e520973"

// MARK: - Properties
bool deviceConnected = false;
bool pressureSensorsConnected = false;
bool imuConnected = false;
unsigned short accelGyroSampleRate = 64;
unsigned short pressureSensorSampleRate = 64;
unsigned short magSampleRate = 64;
bool start = true;

BLEServer *pV2Server;

// MARK: - Sensors Service
BLEService *pSensorsService;
BLECharacteristic *pTopSensorsCharacteristic;
BLECharacteristic *pBottomSensorsCharacteristic;
BLECharacteristic *pPressureSensorConfigurationCharacteristic;

// MARK: - MPU9250 Service
BLEService *pMPU9250Service;
BLECharacteristic *pIMUConfigurationCharacteristic;
BLECharacteristic *pYPRCharacteristic;
BLECharacteristic *pMagnetometerCharacteristic;
BLECharacteristic *pAccelerometerCharacteristic;
BLECharacteristic *pGyroscopeCharacteristic;

// MARK: - Haptics Service
BLEService *pHapticsService;
BLECharacteristic *pHapticsCharacteristic;

// MARK: - Enums
enum BLESensorUUID {
    pressureSensorConfig,
    topSensors,
    bottomSensors,
    imuConfig,
    yprSensors,
    magnetometerSensor,
    accelerometerSensor,
    gyroscopeSensor,
    haptics,
    none
};

BLESensorUUID resolveUUID(std::string input)
{
    // Pressure Sensors
    if (input == PRESSURE_SENSOR_CONFIGURATION_CHARACTERISTIC_UUID) return pressureSensorConfig;
    if (input == TOP_SENSORS_CHARACTERISTIC_UUID) return topSensors;
    if (input == BOTTOM_SENSORS_CHARACTERISTIC_UUID) return bottomSensors;

    // IMU
    if (input == IMU_CONFIGURATION_CHARACTERISTIC_UUID) return imuConfig;
    if (input == YPR_CHARACTERISTIC_UUID) return yprSensors;
    if (input == MAGNETOMETER_CHARACTERISTIC_UUID) return magnetometerSensor;
    if (input == ACCELEROMETER_CHARACTERISTIC_UUID) return accelerometerSensor;
    if (input == GYROSCOPE_CHARACTERISTIC_UUID) return gyroscopeSensor;
    
    // Haptics
    if (input == HAPTICS_CHARACTERISTIC_UUID) return haptics;

    return none;
}

enum IMUGesture: uint8_t { 
    noTap = 1,
    singleTap = 2,
    doubleTap = 3,
    defaultTap
};

void startIMU(bool on, unsigned short agRate, unsigned short mRate)
{
    if (on)
    {
        if (imu.begin() != INV_SUCCESS)
        {
            while (1)
            {
                Serial.println("Unable to communicate with MPU-9250");
                Serial.println("Check connections, and try again");
                Serial.println();
                delay(3000);
            }
        }

        imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

        // Use setGyroFSR() and setAccelFSR() to configure the
        // gyroscope and accelerometer full scale ranges.
        // Gyro options are +/- 250, 500, 1000, or 2000 dps
        imu.setGyroFSR(2000); // Set gyro to 2000 dps
        // Accel options are +/- 2, 4, 8, or 16 g
        imu.setAccelFSR(2); // Set accel to +/-2g
        // Note: the MPU-9250's magnetometer FSR is set at 
        // +/- 4912 uT (micro-tesla's)

        // setLPF() can be used to set the digital low-pass filter
        // of the accelerometer and gyroscope.
        // Can be any of the following: 188, 98, 42, 20, 10, 5
        // (values are in Hz).
        imu.setLPF(5); // Set LPF corner frequency to 5Hz

        // The sample rate of the accel/gyro can be set using
        // setSampleRate. Acceptable values range from 4Hz to 1kHz
        imu.setSampleRate(agRate); // Set sample rate to 10Hz (4 - 1000)

        // Likewise, the compass (magnetometer) sample rate can be
        // set using the setCompassSampleRate() function.
        // This value can range between: 1-100Hz (1 - 100 Hz)
        imu.setCompassSampleRate(mRate); // Set mag rate to 10Hz
    }
    else
    {
        imu.setSensors(0);
    }
}

unsigned short resolveIMUSampleRate(uint8_t sampleRate, bool accelGyro)
{
    switch (sampleRate) 
    {
        case 1:
            return accelGyro ? 32 : 10;
        case 2:
            return accelGyro ? 64 : 20;
        case 3:
            return accelGyro ? 128 : 40;
        case 4:
            return accelGyro ? 256 : 60;
        case 5:
            return accelGyro ? 512 : 80;
        default: 
            return accelGyro ? 10 : 10;
    }
}

unsigned short resolvePressureSensorSampleRate(uint8_t sampleRate)
{
    switch (sampleRate) 
    {
        case 1:
            return 32;
        case 2:
            return 64;
        case 3:
            return 128;
        case 4:
            return 256;
        case 5:
            return 512;
        default: 
            return 64;
    }
}

IMUGesture resolveIMUGesture(uint8_t gesture)
{
    switch (gesture)
    {
        case 1:
            return noTap;
        case 2:
            return singleTap;
        case 3:
            return doubleTap;
        default:
            return defaultTap;
    }
}

inv_error_t updatePressureSensorConfiguration(uint8_t* configuration) 
{
    if (getLength(configuration) == 1) 
    {
        if (configuration[0] == 1)
        {
            pressureSensorsConnected = false;
        } 
        else if (configuration[0] == 2)
        {
            pressureSensorsConnected = true;
        }
        Serial.printf("Pressure sensors are %s\n", pressureSensorsConnected ? "on" : "off");
        return INV_SUCCESS;
    }
    else if (getLength(configuration) == 2)
     {
        if (configuration[0] == 1)
        {
            pressureSensorsConnected = false;
        } 
        else if (configuration[0] == 2)
        {
            pressureSensorsConnected = true;
        }
        pressureSensorSampleRate = resolvePressureSensorSampleRate(configuration[1]);

        Serial.printf("Pressure sensors are %s\n", pressureSensorsConnected ? "on" : "off");
        Serial.printf("Sample Rate: %d\n", pressureSensorSampleRate);

        return INV_SUCCESS;
    }

    Serial.println("Pressure Sensor `configuration` needs to be of length 2");
    return INV_ERROR;
}

inv_error_t updateIMUSensorConfiguration(uint8_t* configuration) 
{

    if (getLength(configuration) == 1)
    {
        if (configuration[0] == 1)
        {
            imuConnected = false;
        } 
        else if (configuration[0] == 2)
        {
            imuConnected = true;
        }
        Serial.printf("IMU is %s\n", imuConnected ? "on" : "off");
        startIMU(imuConnected, 0, 0);
        return INV_SUCCESS;
    }
    else if (getLength(configuration) == 3) {
        if (configuration[0] == 1)
        {
            imuConnected = false;
        } 
        else if (configuration[0] == 2)
        {
            imuConnected = true;
        }
        accelGyroSampleRate = resolveIMUSampleRate(configuration[1], true);
        magSampleRate = resolveIMUSampleRate(configuration[2], false);

        Serial.printf("IMU is %s\n", imuConnected ? "on" : "off");
        Serial.printf("Accel/Gyro Sample Rate: %d\n", accelGyroSampleRate);
        Serial.printf("Mag Sample Rate: %d\n", magSampleRate);

        startIMU(imuConnected, accelGyroSampleRate, magSampleRate);
        return INV_SUCCESS;
    }

    Serial.println("IMU Sensor `configuration` needs to be of length 2");
    return INV_ERROR;
}

inv_error_t updateHapticsConfiguration(uint8_t* configuration) 
{
    uint8_t length = getLength(configuration);
    if (length < 1 || length > lastSlot) 
    {
        Serial.println("Could not set waveform for haptics event.");
        Serial.printf("Haptics motor `configuration` length needs to be at least 1 and less than %d\n", lastSlot);
        return INV_ERROR;
    }
    else
    {
        for (uint8_t i = 0; i < length; i++)
        {
            drv.setWaveform(i, configuration[i]);
        }
        drv.setWaveform(length, 0);
        drv.go();
        delay(hapticsSampleRate);
        Serial.println("Successfully played haptics event.");
        return INV_SUCCESS;
    }
}

// MARK: - Objects
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
    };
};

class MyCallbacks: public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic *pCharacteristic) { 

        std::string uuidString = pCharacteristic->getUUID().toString();
        switch(resolveUUID(uuidString))
        {
            /**
                 * Pressure Sensor Configuration
                 *  1. isConnected
                 *      on:  0x01
                 *      off: 0x02
                 *  2. sampleRate (1-5 from verySlow to veryFast)
                 *      1: verySlow  0x01 -> 320ms
                 *      2: slow      0x02 -> 240ms
                 *      3: medium    0x03 -> 160ms
                 *      4: fast      0x04 ->  80ms
                 *      5: veryFast  0x05 ->  40ms
                 **/
            case pressureSensorConfig:
            {
                Serial.println("Attempting to write to `PRESSURE_SENSORS_CONFIGURATION_CHARACTERISTIC_UUID`");
                uint8_t* data = pCharacteristic->getData();

                inv_error_t pscError = updatePressureSensorConfiguration(data);

                if (pscError == INV_SUCCESS)
                {
                    Serial.println("Successfully updated pressure sensor configuration...");
                }
                else if (pscError == INV_ERROR)
                {
                    printByteArray(data);   
                }
                
                break;
            }
            case imuConfig:
            {
                /**
                 * IMU Configuration
                 *  1. isConnected
                 *      off: 0x01
                 *      on:  0x02
                 *  2. accelGyroSampleRate (1-5 from verySlow to veryFast)
                 *      1: verySlow  0x01 -> 512Hz
                 *      2: slow      0x02 -> 256Hz
                 *      3: medium    0x03 -> 128Hz
                 *      4: fast      0x04 ->  64Hz
                 *      5: veryFast  0x05 ->  32Hz
                 *  3. magSampleRate (1-5 from verySlow to veryFast)
                 *      1: verySlow  0x01 -> 80Hz
                 *      2: slow      0x02 -> 60Hz
                 *      3: medium    0x03 -> 40Hz
                 *      4: fast      0x04 ->  20Hz
                 *      5: veryFast  0x05 ->  10Hz
                 **/
                Serial.println("Attempting to write to `IMU_CONFIGURATION_CHARACTERISTIC_UUID`");
                uint8_t* data = pCharacteristic->getData();
                inv_error_t imuError = updateIMUSensorConfiguration(data);

                if (imuError == INV_SUCCESS)
                {
                    Serial.println("Successfully updated IMU sensor configuration...");
                }
                else if (imuError == INV_ERROR)
                {
                    printByteArray(data);   
                }
                break;
            }
            case haptics: 
            {
                /**
                 * Haptics Configuration
                 *  Slots 0...7 (Total of 8 slots) 
                 *      slot 0: hex:0x00 (decimal:0) - hex:0x7B (decimal:123)
                 *      slot 1: hex:0x00 (decimal:0) - hex:0x7B (decimal:123)
                 *      ...
                 *      Slot 8: hex:0x00 (decimal:0) - hex:0x7B (decimal:123)
                 *  ** Note: there may be fewer than 8 slots in the haptics configuration
                 **/
                Serial.println("Attempting to write to `HAPTICS_CHARACTERISTIC_UUID`"); 
                uint8_t* data = pCharacteristic->getData();
                inv_error_t hapticsError = updateHapticsConfiguration(data);
                if (hapticsError == INV_SUCCESS)
                {
                    Serial.println("Successfully fired haptics event..");
                }
                else if (hapticsError == INV_ERROR)
                {
                    printByteArray(data);   
                }
                break;
            }
            case none:
                Serial.println("No matching UUID was present.");     
                break;  
            default: 
                break;
        }
    }
};

// MARK: - Functions
void v2BLESetup(std::string deviceName) {
    // Create the BLE Device
    BLEDevice::init(deviceName);
    // Create the BLE Server
    pV2Server = BLEDevice::createServer();
    pV2Server->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    pSensorsService = pV2Server->createService(SERVICE_UUID_1);
    pMPU9250Service = pV2Server->createService(SERVICE_UUID_2);
    pHapticsService = pV2Server->createService(SERVICE_UUID_3);

    pPressureSensorConfigurationCharacteristic = pSensorsService->createCharacteristic(
                                            PRESSURE_SENSOR_CONFIGURATION_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pPressureSensorConfigurationCharacteristic->setCallbacks(new MyCallbacks());

    pTopSensorsCharacteristic = pSensorsService->createCharacteristic(
                                            TOP_SENSORS_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                        );
    pTopSensorsCharacteristic->setCallbacks(new MyCallbacks());


    pBottomSensorsCharacteristic = pSensorsService->createCharacteristic(
                                            BOTTOM_SENSORS_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
                                        );
    pBottomSensorsCharacteristic->setCallbacks(new MyCallbacks());

    pIMUConfigurationCharacteristic = pMPU9250Service->createCharacteristic(
                                            IMU_CONFIGURATION_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY

    );
    pIMUConfigurationCharacteristic->setCallbacks(new MyCallbacks());

    pYPRCharacteristic = pMPU9250Service->createCharacteristic(
                                           YPR_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pYPRCharacteristic->setCallbacks(new MyCallbacks());

    pMagnetometerCharacteristic = pMPU9250Service->createCharacteristic(
                                           MAGNETOMETER_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pMagnetometerCharacteristic->setCallbacks(new MyCallbacks());

    pAccelerometerCharacteristic = pMPU9250Service->createCharacteristic(
                                           ACCELEROMETER_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pAccelerometerCharacteristic->setCallbacks(new MyCallbacks());


    pGyroscopeCharacteristic = pMPU9250Service->createCharacteristic(
                                           GYROSCOPE_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pGyroscopeCharacteristic->setCallbacks(new MyCallbacks());

    pHapticsCharacteristic = pHapticsService->createCharacteristic(
                                           HAPTICS_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pHapticsCharacteristic->setCallbacks(new MyCallbacks());


    // BLE2902 needed to notify
    pPressureSensorConfigurationCharacteristic->addDescriptor(new BLE2902());
    pTopSensorsCharacteristic->addDescriptor(new BLE2902());
    pBottomSensorsCharacteristic->addDescriptor(new BLE2902());
   
    pIMUConfigurationCharacteristic->addDescriptor(new BLE2902());
    pYPRCharacteristic->addDescriptor(new BLE2902());
    pMagnetometerCharacteristic->addDescriptor(new BLE2902());
    pAccelerometerCharacteristic->addDescriptor(new BLE2902());
    pGyroscopeCharacteristic->addDescriptor(new BLE2902());

    pHapticsCharacteristic->addDescriptor(new BLE2902());

    // Start services
    pSensorsService->start();
    pMPU9250Service->start();
    pHapticsService->start();

    // Start advertising
    pV2Server->getAdvertising()->start();
    Serial.println("Waiting for a client connection to notify...");
}
