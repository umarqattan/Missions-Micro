
// MARK: - Defines
#define IS_LEFT //IS_RIGHT 
#define DEBUG_BLE_SENSORS

// MARK: - MUX variables
enum Sensors { top, bottom };

uint16_t muxIntArray[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
char muxTopByteArray[20];
char muxBottomByteArray[12];

int topSensorsCount = 10;
int bottomSensorsCount = 6;
int s0 = 4;
int s1 = 12;
int s2 = 15;
int s3 = 25;
int SIG_pin = 36; 

int controlPin[] = {s0, s1, s2, s3}; 

#ifdef IS_LEFT
int muxChannel[16][4]= { 
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
int muxChannel[16][4]={ 
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
int readMux(int channel);
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

int readMux(int channel)
{     
  for(int i = 0; i < 4; i ++)
  { 
    digitalWrite(controlPin[i], muxChannel[channel][i]); 
  } 
  
  int val = analogRead(SIG_pin);
  return val; 
}

void printMux()
{
  Serial.print("[");
  for(int i = 0; i < 16; i++)
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