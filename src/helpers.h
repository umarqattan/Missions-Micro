#include <Arduino.h>
#include <HardwareSerial.h>
#include <lwipopts.h>
#include "Wire.h"

#define FRACTIONAL_BITS 5

void initByteArray(uint8_t *var, uint8_t length);
void scanner();
uint8_t getLength(uint8_t *byteArray);
void printByteArray(uint8_t *byteArray);
void printPressureByteArray(uint8_t *sensorByteArray);
void printIMUByteArray(uint8_t* imuByteArray, uint8_t size);
void printQuatArray(float *quatArray, uint8_t size);
uint8_t truncate(uint16_t value);
int16_t floatToFixed(float value);
float fixedToFloat(int16_t value);


void initByteArray(uint8_t *var, uint8_t length)
{
    for (uint8_t i = 0; i < length; i++)
    {
        var[i] = 0;
    }
}

/* 
    Will truncate a number by shifting the bits by 4 places, as this acts as a map from uint16_t
    to uint8_t. Precision during truncation will be lost, but the error is almost negligible.

    value = 2100
    truncate(value)
        = (uint8_t)(0b100000110100 >> 4)
        = (uint8_t)(0b10000011)
        = 131
 */
uint8_t truncate(uint16_t value)
{
    return (uint8_t)(value >> 4);
}

int16_t floatToFixed(float value)
{
    return (int16_t)(round(value * (1 << FRACTIONAL_BITS)));
}

float fixedToFloat(int16_t value)
{
    return (float) (value / ((float)(1 << FRACTIONAL_BITS)));
}

void scanner()
{
    Serial.println();
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    Wire.begin(21, 22, 100000);
    for (byte i = 8; i < 120; i++)
    {
        Wire.beginTransmission(i);       // Begin I2C transmission Address (i)
        if (Wire.endTransmission() == 0) // Receive 0 = success (ACK response)
        {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX); // PCF8574 7 bit address
            Serial.println(")");
            count++;
        }
    }
    Serial.print("Found ");
    Serial.print(count, DEC); // numbers of devices
    Serial.println(" device(s).");
}

uint8_t getLength(uint8_t *byteArray)
{
    uint8_t i = 0;
    while (byteArray[i] != '\0')
    {
        i += 1;
    }

    return i;
}

void printPressureByteArray(uint8_t *byteArray)
{
    uint8_t bottomSensorsCount = 6;
    uint8_t topSensorsCount = 10;

    Serial.println("Bottom");
    Serial.print("[ ");
    for (uint8_t i = 0; i < 2*bottomSensorsCount; i++)
    {
        Serial.printf("%d ", byteArray[i+1]);
    }
    Serial.println("]");

    Serial.println("Top");
    Serial.print("[ ");
    for (uint8_t i = 0; i < 2*topSensorsCount; i++)
    {
        Serial.printf("%d ", byteArray[i + 2*bottomSensorsCount + 1]);
    }
    Serial.println("]");

    Serial.println("Time");
    Serial.print("[ ");
    for (uint8_t i = 0; i < 4; i++)
    {
        Serial.printf("%d ", byteArray[i + 2*bottomSensorsCount + 2*topSensorsCount + 1]);
    }
    Serial.println("]");

}

void printQuatArray(float *quatArray, uint8_t size)
{
    Serial.printf("[ ");
    for (uint8_t i = 0; i < size; i++)
    {
        Serial.printf("%f ", quatArray[i]);
    }
    Serial.println("]"); 
}

void printIMUByteArray(uint8_t *imuByteArray, uint8_t size)
{
    Serial.printf("[ ");
    for (uint8_t i = 0; i < size; i++)
    {
        Serial.printf("%d ", imuByteArray[i]);
    }
    Serial.println("]");    
}


void printByteArray(uint8_t *byteArray)
{
    uint8_t i = 0;
    Serial.printf("Byte Array:");
    Serial.printf("[");
    while (byteArray[i] != '\0')
    {
        if (byteArray[i + 1] == '\0')
        {
            Serial.printf("%d]", byteArray[i]);
        }
        else
        {
            Serial.printf("%d, ", byteArray[i]);
        }
        i += 1;
    }
}