// MARK: - Includes
#include "HardwareSerial.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "mpu.h"
#include "haptics.h"

// V2


// Service 1: IMU Data Characteristics
#define SERVICE_UUID_1 "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define ACCELEROMETER_DATA_CHARACTERISTIC_UUID "5cf06fe8-ed01-4e1a-969e-126bc70b0950"
#define GYROSCOPE_DATA_CHARACTERISTIC_UUID "3938f998-070e-4bd7-a15f-1925e7afef9f"
#define QUATERNION_DATA_CHARACTERISTIC_UUID "96e8a211-f97f-4349-9a31-0437ecd43cdd"

// Service 2: Other Characteristics
#define SERVICE_UUID_2 "31408399-730b-4d53-911e-993cd531e96f"
#define HAPTICS_CHARACTERISTIC_UUID "ff227c5f-bfef-4db4-8ce0-e4ef8e520973"
#define PRESSURE_DATA_CHARACTERISTIC_UUID "5d11a41a-8479-47cd-bd96-8e04dc6e98a2"
// #define LINEAR_ACCELERATION_DATA_CHARACTERISTIC_UUID "f3041b0d-170b-4e47-b7ad-b30368912e06"

// Service 3: Configuration Characteristics
#define SERVICE_UUID_3 "0bb6d9ac-1012-448b-8485-495bea5238a7"
#define CONFIGURATION_CHARACTERISTIC_UUID "5a6256f0-fc1c-42b5-ba7b-6585f39cfc7e"

// MARK: - Properties
bool deviceConnected = false;
bool imuConnected = false;
bool pressureConnected = false;
unsigned char accOn = false;
unsigned char gyroOn = false;
unsigned char quatOn = false;
unsigned char pressureOn = false;


uint8_t mtuValue = 61;
bool start = true;

BLEServer *pV2Server;
// MARK: - IMU Service
BLEService *pIMUDataService;
BLECharacteristic *pAccelerometerDataCharacteristic;
BLECharacteristic *pGyroscopeDataCharacteristic;
BLECharacteristic *pQuaternionDataCharacteristic;


// MARK: - Other Sensors Service
BLEService *pOtherService;
BLECharacteristic *pHapticsCharacteristic;
BLECharacteristic *pPressureDataCharacteristic;
// BLECharacteristic *pLinearAccelerometerDataCharacteristic;

// MARK: - Configuration Service
BLEService *pConfigurationService;
BLECharacteristic *pConfigurationCharacteristic; 

// MARK: - Enums
enum mBLEUUID {
    config = 1,
    data,
    haptics,
    none
};

enum mBLEDataSource { 
    topSensor = 1,
    bottomSensor,
    accelerometer,
    gyroscope,
    magnetometer,
    noSource
};


mBLEDataSource resolveDataSource(uint8_t source)
{
    switch (source)
    {
        case topSensor: return topSensor;
        case bottomSensor: return bottomSensor;
        case accelerometer: return accelerometer;
        case gyroscope: return gyroscope;
        case magnetometer: return magnetometer;
        default: return noSource;
    }
}

mBLEUUID resolveUUID(std::string input)
{
    if (input == CONFIGURATION_CHARACTERISTIC_UUID) return config;
    if (input == HAPTICS_CHARACTERISTIC_UUID) return haptics;

    return none;
}

void updateIMU()
{
    unsigned char on = accOn | gyroOn | quatOn;

    if (on)
    {
        if (!imuConnected)
        {
            if (imu.begin() != INV_SUCCESS)
            {
                while (1)
                {
                    Serial.println("Unable to communicate with MPU-9250");
                    Serial.println("Check connections, and try again");
                    Serial.println();
                    delay(5000);
                }
            }

            imu.setSensors(INV_XYZ_GYRO | INV_WXYZ_QUAT | INV_XYZ_ACCEL);
            imu.setGyroFSR(2000); // Set gyro to 2000 dps
            imu.setLPF(5); // Set LPF corner frequency to 5Hz

            // The sample rate of the accel/gyro can be set using
            // setSampleRate. Acceptable values range from 4Hz to 1kHz
            // imu.setSampleRate(50); // Set sample rate to 10Hz (4 - 1000)

            imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_RAW_GYRO,
                50);  // 50 Hz == 20 ms (matches delay)
            imuConnected = true;
        }
    }
    else
    {
        imu.dmpBegin(0);
        imuConnected = false;
    }
    Serial.printf("imuConnected: %d\n", imuConnected);
}

unsigned short resolvePressureSensorSampleRate(uint8_t sampleRate)
{
    switch (sampleRate) 
    {
        case 0:
            return 40;
        case 1:
            return 80;
        case 2:
            return 160;
        case 3:
            return 240;
        default:
            return 500;
    }
}

unsigned short resolveIMUSampleRate(uint8_t sampleRate, bool accelGyro)
{
    switch (sampleRate) 
    {
        case 1:
            return accelGyro ? 500 : 10;
        case 2:
            return accelGyro ? 750 : 50;
        case 3:
            return accelGyro ? 1000 : 100;
        default: 
            return accelGyro ? 500 : 10;
    }
}

void updateDataProvider(uint8_t* configuration)
{
    uint8_t length = getLength(configuration);
    if (length == 1)
    {
        uint8_t byte = configuration[0];
        if (byte == 1)
        {
            pressureOn = 0;
            accOn = 0;
            gyroOn = 0;
            quatOn = 0;
        } 
        else 
        {
            pressureOn = (unsigned char)((byte & (1 << 4)) >> 4);
            accOn = (unsigned char)((byte & (1 << 5)) >> 5);
            gyroOn = (unsigned char)((byte & (1 << 6)) >> 6);
            quatOn = (unsigned char)((byte & (1 << 7)) >> 7);
        }

        updateIMU();        
    }
    else 
    {
        Serial.printf("incorrect length %d: getLength(configuration) != 1", length);
    }
}

void updateHapticsConfiguration(uint8_t* configuration) 
{
    uint8_t length = getLength(configuration);
    if (length < 1 || length > lastSlot) 
    {
        Serial.println("Could not set waveform for haptics event.");
        Serial.printf("Haptics motor `configuration` length needs to be at least 1 and less than %d\n", lastSlot);
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

            case config:
            {
                Serial.println("Attempting to write to `CONFIGURATION_CHARACTERISTIC_UUID`");
                uint8_t* configuration = pCharacteristic->getData();
                updateDataProvider(configuration);
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
                updateHapticsConfiguration(data);
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
    pIMUDataService = pV2Server->createService(SERVICE_UUID_1);
    pOtherService = pV2Server->createService(SERVICE_UUID_2);
    pConfigurationService = pV2Server->createService(SERVICE_UUID_3);
    
    pAccelerometerDataCharacteristic = pIMUDataService->createCharacteristic(
                                            ACCELEROMETER_DATA_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pAccelerometerDataCharacteristic->setCallbacks(new MyCallbacks());

    pGyroscopeDataCharacteristic = pIMUDataService->createCharacteristic(
                                            GYROSCOPE_DATA_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pGyroscopeDataCharacteristic->setCallbacks(new MyCallbacks());

    pQuaternionDataCharacteristic = pIMUDataService->createCharacteristic(
                                            QUATERNION_DATA_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pQuaternionDataCharacteristic->setCallbacks(new MyCallbacks());


    pPressureDataCharacteristic = pOtherService->createCharacteristic(
                                            PRESSURE_DATA_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pPressureDataCharacteristic->setCallbacks(new MyCallbacks()); 


    // pLinearAccelerometerDataCharacteristic = pOtherService->createCharacteristic(
    //                                         LINEAR_ACCELERATION_DATA_CHARACTERISTIC_UUID,
    //                                         BLECharacteristic::PROPERTY_READ |
    //                                         BLECharacteristic::PROPERTY_WRITE |
    //                                         BLECharacteristic::PROPERTY_NOTIFY
    // );
    // pLinearAccelerometerDataCharacteristic->setCallbacks(new MyCallbacks()); 

    pHapticsCharacteristic = pOtherService->createCharacteristic(
                                           HAPTICS_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pHapticsCharacteristic->setCallbacks(new MyCallbacks());

    pConfigurationCharacteristic = pConfigurationService->createCharacteristic(
                                            CONFIGURATION_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pConfigurationCharacteristic->setCallbacks(new MyCallbacks());

    // BLE2902 needed to notify
    BLEDescriptor *accelerometerDataDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    accelerometerDataDescriptor->setValue("Accelerometer Data");
    pAccelerometerDataCharacteristic->addDescriptor(accelerometerDataDescriptor);
    pAccelerometerDataCharacteristic->addDescriptor(new BLE2902());

    BLEDescriptor *gyroscopeDataDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    gyroscopeDataDescriptor->setValue("Gyroscope Data");
    pGyroscopeDataCharacteristic->addDescriptor(gyroscopeDataDescriptor);
    pGyroscopeDataCharacteristic->addDescriptor(new BLE2902());

    BLEDescriptor *quaternionDataDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    quaternionDataDescriptor->setValue("Quaternion Data");
    pQuaternionDataCharacteristic->addDescriptor(quaternionDataDescriptor);
    pQuaternionDataCharacteristic->addDescriptor(new BLE2902());

    BLEDescriptor *pressureDataDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    pressureDataDescriptor->setValue("Pressure Data");
    pPressureDataCharacteristic->addDescriptor(pressureDataDescriptor);
    pPressureDataCharacteristic->addDescriptor(new BLE2902());

    // BLEDescriptor *linearAccelerationDataDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    // linearAccelerationDataDescriptor->setValue("Linear Accceleration Data");
    // pLinearAccelerometerDataCharacteristic->addDescriptor(linearAccelerationDataDescriptor);
    // pLinearAccelerometerDataCharacteristic->addDescriptor(new BLE2902());

    BLEDescriptor *configurationDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    configurationDescriptor->setValue("Configuration");
    pConfigurationCharacteristic->addDescriptor(configurationDescriptor);
    pConfigurationCharacteristic->addDescriptor(new BLE2902());
    
    BLEDescriptor *hapticsDescription = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    hapticsDescription->setValue("Haptics");
    pHapticsCharacteristic->addDescriptor(hapticsDescription);
    pHapticsCharacteristic->addDescriptor(new BLE2902());
    
    // Start services
    pIMUDataService->start();
    pOtherService->start();
    pConfigurationService->start();

    // Start advertising
    pV2Server->getAdvertising()->start();
    Serial.println("Waiting for a client connection to notify...");
}
