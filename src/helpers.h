#include <Arduino.h>
#include <HardwareSerial.h>
#include <lwipopts.h>
#include "Wire.h"

void initByteArray(char *var, int length);
void scanner();
uint8_t getLength(uint8_t* byteArray);

void initByteArray(char *var, int length)
{
  for (int i = 0; i < length; i++)
  {
    var[i] = 0;
  }
}

void scanner()
{
  Serial.println ();
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;

  Wire.begin(21, 22, 100000);
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
}

uint8_t getLength(uint8_t* byteArray)
{
  uint8_t i = 0;
  while(byteArray && byteArray[i++] != '\0') {};

  return i;
}