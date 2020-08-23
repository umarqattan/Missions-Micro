
// MARK: - Defines
#define IS_RIGHT
#define DEBUG_BLE_SENSORS
#define PRESSURE_SIZE 37
#define TOP_SIZE 20
#define BOTTOM_SIZE 12

#define PRESSURE_ON 0x1 << 4

// MARK: - MUX variables
enum Sensors { top, bottom };


uint8_t muxByteArray[PRESSURE_SIZE];
uint8_t muxTopByteArray[TOP_SIZE];
uint8_t muxBottomByteArray[BOTTOM_SIZE];

uint8_t topSensorsCount = 10;
uint8_t bottomSensorsCount = 6;
uint8_t sensorsCount = 16;
uint8_t s0 = 4;
uint8_t s1 = 12;
uint8_t s2 = 15;
uint8_t s3 = 25;
uint8_t SIG_pin = 36; 

uint8_t controlPin[] = {s0, s1, s2, s3}; 

#ifdef IS_LEFT
uint8_t muxChannel[16][4]= { 
    {1,1,1,0}, //channel 7 
    {0,1,1,0}, //channel 6 
    {1,0,1,0}, //channel 5 
    {0,0,1,0}, //channel 4 
    {1,1,0,0}, //channel 3    
    {0,1,0,0}, //channel 2  
    {1,0,0,0}, //channel 1 
    {0,0,0,0}, //channel 0 
    {1,1,1,1}, //channel 15
    {0,1,1,1}, //channel 14
    {1,0,1,1}, //channel 13
    {0,0,1,1}, //channel 12
    {1,1,0,1}, //channel 11
    {0,1,0,1}, //channel 10
    {1,0,0,1}, //channel 9     
    {0,0,0,1}  //channel 8 
};
#endif

#ifdef IS_RIGHT
uint8_t muxChannel[16][4]={ 
    {1,0,1,1}, //channel 13
    {0,0,1,1}, //channel 12
    {1,1,0,1}, //channel 11
    {0,1,0,1}, //channel 10
    {1,0,0,1}, //channel 9     
    {0,0,0,1}, //channel 8 
    {1,1,1,0}, //channel 7 
    {0,1,1,0}, //channel 6 
    {1,0,1,0}, //channel 5 
    {0,0,1,0}, //channel 4 
    {1,1,0,0}, //channel 3    
    {0,1,0,0}, //channel 2  
    {1,0,0,0}, //channel 1 
    {0,0,0,0}, //channel 0 
    {1,1,1,1}, //channel 15
    {0,1,1,1}  //channel 14
};
#endif

void setupMux();
uint16_t readMux(uint8_t channel);
void sensorMuxLoop();
void printMux();

void setupMux()
{
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
}

uint16_t readMux(uint8_t channel)
{     
  for(uint8_t i = 0; i < 4; i ++)
  { 
    digitalWrite(controlPin[i], muxChannel[channel][i]); 
  } 
  
  uint16_t val = analogRead(SIG_pin);
  return val; 
}

void printMux()
{
  Serial.print("[");
  for(uint8_t i = 0; i < 16; i++)
  { 
    if (i < 15) 
    {
      Serial.print(readMux(i));
      Serial.print(",");
    } else {
      Serial.print(readMux(i));
      Serial.println("]");
    }
  } 
}