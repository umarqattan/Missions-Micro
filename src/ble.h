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
#define SERVICE_UUID_2 "31408399-730b-4d53-911e-993cd531e96f"

#define CONFIGURATION_CHARACTERISTIC_UUID "5a6256f0-fc1c-42b5-ba7b-6585f39cfc7e"
#define DATA_CHARACTERISTIC_UUID "7f922bbb-74d8-45a4-833f-e3bdbcfca5a2"

#define HAPTICS_CHARACTERISTIC_UUID "ff227c5f-bfef-4db4-8ce0-e4ef8e520973"

// MARK: - Properties
bool deviceConnected = false;
bool pressureSensorsConnected = false;
bool imuConnected = false;
unsigned char imuSensors = UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS;
unsigned char imuBitConfiguration = 0;
unsigned char pressureBitConfiguration = 0;
unsigned char sensorConfiguration = 0;
unsigned short accelGyroSampleRate = 64;
unsigned short pressureSensorSampleRate = 64;
unsigned short magSampleRate = 64;
uint8_t mtuValue = 30;
bool start = true;

BLEServer *pV2Server;
// MARK: - Data Service
BLEService *pDataService;
BLECharacteristic *pDataCharacteristic;
BLECharacteristic *pConfigurationCharacteristic; 

// MARK: - Haptics Service
BLEService *pHapticsService;
BLECharacteristic *pHapticsCharacteristic;

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
    if (input == DATA_CHARACTERISTIC_UUID) return data;
    if (input == HAPTICS_CHARACTERISTIC_UUID) return haptics;

    return none;
}

void startIMU(unsigned char accelBit, unsigned char gyroBit, unsigned char magBit)
{
    unsigned char on = accelBit | gyroBit | magBit;
    Serial.printf("accelBit: %d\n", accelBit);
    Serial.printf("gyroBit: %d\n", gyroBit);
    Serial.printf("magBit: %d\n", magBit);

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

        imu.setSensors(accelBit | gyroBit | magBit);
        
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
        imu.setSampleRate(accelGyroSampleRate); // Set sample rate to 10Hz (4 - 1000)

        // Likewise, the compass (magnetometer) sample rate can be
        // set using the setCompassSampleRate() function.
        // This value can range between: 1-100Hz (1 - 100 Hz)
        imu.setCompassSampleRate(magSampleRate); // Set mag rate to 10Hz
        imuConnected = true;
    }
    else
    {
        imu.setSensors(0);
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
            return accelGyro ? 50 : 25;
        case 2:
            return accelGyro ? 250 : 50;
        case 3:
            return accelGyro ? 500 : 100;
        default: 
            return accelGyro ? 10 : 10;
    }
}

void configurePressureSensors(uint8_t sampleRate, uint8_t pressureBit)
{
    Serial.printf("pressureSensorsConnected: %d\n", pressureBit);
    pressureSensorsConnected = pressureBit;
    pressureSensorSampleRate = resolvePressureSensorSampleRate(sampleRate);
}

void configureIMU(uint8_t sampleRate, uint8_t accelBit, uint8_t gyroBit, uint8_t magBit)
{
    unsigned char shouldUpdateAccel = accelBit ? UPDATE_ACCEL : 0;
    unsigned char shouldUpdateGyro = gyroBit ? UPDATE_GYRO : 0;
    unsigned char shouldUpdateMag = magBit ? UPDATE_COMPASS : 0;
    imuSensors = shouldUpdateAccel | shouldUpdateGyro | shouldUpdateMag;
    Serial.printf("imuSensors: %d\n", imuSensors);
    accelGyroSampleRate = resolveIMUSampleRate(sampleRate, true);
    magSampleRate = resolveIMUSampleRate(sampleRate, false);
    startIMU(accelBit, gyroBit, magBit);
}

void updateDataProvider(uint8_t* configuration)
{
    uint8_t length = getLength(configuration);
    if (length == 1)
    {
        uint8_t byte = configuration[0];
        uint8_t pressureBit = (byte & (1 << 4)) >> 4;

        unsigned char accelConfigBit = (unsigned char)((byte & (1 << 5)) >> 5);
        unsigned char accelBit = accelConfigBit ? INV_XYZ_ACCEL : 0;

        unsigned char gyroConfigBit = (unsigned char)((byte & (1 << 6)) >> 6);
        unsigned char gyroBit = gyroConfigBit ? INV_XYZ_GYRO : 0;

        unsigned char magConfigBit = (unsigned char)((byte & (1 << 7)) >> 7);
        unsigned char magBit = magConfigBit ? INV_XYZ_COMPASS : 0;

        unsigned char imuBitSampleRate = (byte & 0xC) >> 2;
        unsigned char pressureBitSampleRate = byte & 0x3;

        imuBitConfiguration = ((accelConfigBit << 5) | (gyroConfigBit << 6) | (magConfigBit << 7));
        pressureBitConfiguration = (pressureBit << 4);
        sensorConfiguration = (imuBitConfiguration | pressureBitConfiguration) & 0xF0;
        configureIMU(imuBitSampleRate, accelBit, gyroBit, magBit);        
        configurePressureSensors(pressureBitSampleRate, pressureBit);
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
            case data:
            {
                return;
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
    BLEDevice::setMTU(mtuValue);
    // Create the BLE Server
    pV2Server = BLEDevice::createServer();
    pV2Server->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    pDataService = pV2Server->createService(SERVICE_UUID_1);
    pHapticsService = pV2Server->createService(SERVICE_UUID_2);

    pDataCharacteristic = pDataService->createCharacteristic(
                                            DATA_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pDataCharacteristic->setCallbacks(new MyCallbacks());

    pConfigurationCharacteristic = pDataService->createCharacteristic(
                                            CONFIGURATION_CHARACTERISTIC_UUID,
                                            BLECharacteristic::PROPERTY_READ |
                                            BLECharacteristic::PROPERTY_WRITE |
                                            BLECharacteristic::PROPERTY_NOTIFY
    );
    pConfigurationCharacteristic->setCallbacks(new MyCallbacks());

    pHapticsCharacteristic = pHapticsService->createCharacteristic(
                                           HAPTICS_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pHapticsCharacteristic->setCallbacks(new MyCallbacks());

    // BLE2902 needed to notify
    BLEDescriptor *dataDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    dataDescriptor->setValue("Data");
    pDataCharacteristic->addDescriptor(dataDescriptor);
    pDataCharacteristic->addDescriptor(new BLE2902());

    BLEDescriptor *configurationDescriptor = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    configurationDescriptor->setValue("Configuration");
    pConfigurationCharacteristic->addDescriptor(configurationDescriptor);
    pConfigurationCharacteristic->addDescriptor(new BLE2902());
    
    BLEDescriptor *hapticsDescription = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
    hapticsDescription->setValue("Haptics");
    pHapticsCharacteristic->addDescriptor(hapticsDescription);
    pHapticsCharacteristic->addDescriptor(new BLE2902());
    
    // Start services
    pDataService->start();
    pHapticsService->start();

    // Start advertising
    pV2Server->getAdvertising()->start();
    Serial.println("Waiting for a client connection to notify...");
}
