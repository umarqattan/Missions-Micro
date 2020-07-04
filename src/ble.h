// MARK: - Includes
#include "HardwareSerial.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// V2
#define SERVICE_UUID_1 "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define SERVICE_UUID_2 "7a658cba-0dcd-4d02-bb97-80296cf72dfd"
#define SERVICE_UUID_3 "31408399-730b-4d53-911e-993cd531e96f"
#define TOP_SENSORS_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BOTTOM_SENSORS_CHARACTERISTIC_UUID "0031a9bf-b569-4f8c-9d18-2256c9561d83"
#define ACCELEROMETER_CHARACTERISTIC_UUID "067bc721-08ec-4b74-aa3e-c5c7d29b3025"
#define GYROSCOPE_CHARACTERISTIC_UUID "48737c77-8583-4389-9995-a34188c50b4b"
#define MAGNETOMETER_CHARACTERISTIC_UUID "c7e6d447-a0a7-4e22-bf14-c0be89905302"
#define HAPTICS_CHARACTERISTIC_UUID "ff227c5f-bfef-4db4-8ce0-e4ef8e520973"

// MARK: - Properties

bool deviceConnected = false;
int samplingRate = 1000;
bool start = true;

BLEServer *pV2Server;

// MARK: - Sensors Service
BLEService *pSensorsService;
BLECharacteristic *pTopSensorsCharacteristic;
BLECharacteristic *pBottomSensorsCharacteristic;

// MARK: - MPU9250 Service
BLEService *pMPU9250Service;
BLECharacteristic *pAccelerometerCharacteristic;
BLECharacteristic *pGyroscopeCharacteristic;
BLECharacteristic *pMagnetometerCharacteristic;

// MARK: - Haptics Service
BLEService *pHapticsService;
BLECharacteristic *pHapticsCharacteristic;

// MARK: - Enums
enum BLESensorUUID {
    topSensors,
    bottomSensors,
    accelerometer,
    gyroscope,
    magnetometer,
    haptics,
    none
};

BLESensorUUID resolveUUID(std::string input);

BLESensorUUID resolveUUID(std::string input)
{
    if (input == TOP_SENSORS_CHARACTERISTIC_UUID) return topSensors;
    if (input == BOTTOM_SENSORS_CHARACTERISTIC_UUID) return bottomSensors;
    if (input == ACCELEROMETER_CHARACTERISTIC_UUID) return accelerometer;
    if (input == GYROSCOPE_CHARACTERISTIC_UUID) return gyroscope;
    if (input == MAGNETOMETER_CHARACTERISTIC_UUID) return magnetometer;
    if (input == HAPTICS_CHARACTERISTIC_UUID) return haptics;

    return none;
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
        std::string valueString = pCharacteristic->getValue().c_str();
        if (valueString.length() == 0)
        {
            Serial.printf("Invalid value length...");
            return;
        }
        switch(resolveUUID(uuidString))
        {
            case topSensors:
                Serial.printf("Attempting to write to `TOP_SENSORS_CHARACTERISTIC`\n");
                samplingRate = atoi(pCharacteristic->getValue().c_str());
                Serial.printf("New value string %d", samplingRate);
                break;
            case bottomSensors:
                Serial.printf("Attempting to write to `BOTTOM_SENSORS_CHARACTERISTIC_UUID`\n");
                samplingRate = atoi(pCharacteristic->getValue().c_str());
                Serial.printf("New value string %d", samplingRate);
                break;
            case accelerometer:
                Serial.printf("Attempting to write to `ACCELEROMETER_CHARACTERISTIC_UUID`\n");
                samplingRate = valueString[0];
                Serial.printf("New value string %d", samplingRate);
                break;
            case gyroscope:
                Serial.printf("Attempting to write to `GYROSCOPE_CHARACTERISTIC_UUID`\n");  
                samplingRate = valueString[0];
                Serial.printf("New value string %d", samplingRate);  
                break;
            case magnetometer:
                Serial.printf("Attempting to write to `MAGNETOMETER_CHARACTERISTIC_UUID`\n"); 
                samplingRate = valueString[0];
                Serial.printf("New value string %d", samplingRate);   
                break;
            case haptics: {
                effect = pCharacteristic->getData();
                Serial.printf("Attempting to write to `HAPTICS_CHARACTERISTIC_UUID`\n"); 
                break;
            }
            case none:
                Serial.printf("No matching UUID was present.");     
                break;  
            default: 
                break;
        }

        if (samplingRate == 0) 
        {
            deviceConnected = false;
        } 

        if (!deviceConnected && samplingRate >= 50)
        {
            deviceConnected = true;
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

    pMagnetometerCharacteristic = pMPU9250Service->createCharacteristic(
                                           MAGNETOMETER_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pMagnetometerCharacteristic->setCallbacks(new MyCallbacks());

    pHapticsCharacteristic = pHapticsService->createCharacteristic(
                                           HAPTICS_CHARACTERISTIC_UUID,
                                           BLECharacteristic::PROPERTY_READ |
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_NOTIFY
                                         );
    pHapticsCharacteristic->setCallbacks(new MyCallbacks());


    // BLE2902 needed to notify
    pTopSensorsCharacteristic->addDescriptor(new BLE2902());
    pBottomSensorsCharacteristic->addDescriptor(new BLE2902());
    pAccelerometerCharacteristic->addDescriptor(new BLE2902());
    pGyroscopeCharacteristic->addDescriptor(new BLE2902());
    pMagnetometerCharacteristic->addDescriptor(new BLE2902());
    pHapticsCharacteristic->addDescriptor(new BLE2902());

    // Start services
    pSensorsService->start();
    pMPU9250Service->start();
    pHapticsService->start();

    // Start advertising
    pV2Server->getAdvertising()->start();
    Serial.println("Waiting for a client connection to notify...");
}
