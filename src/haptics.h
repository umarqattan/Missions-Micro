// MARK: - Includes
#include "Adafruit_DRV2605.h"


// MARK: - Haptics Driver variables
Adafruit_DRV2605 drv;
uint8_t* effect;
int hapticsSampleRate = 500;
uint8_t lastSlot = 6;
uint8_t lastEffect = 117;


enum HapticsState { 
  idle,
  inProgress,
  incomplete,
  done 
};

HapticsState hapticsState = idle;

// MARK: - Prototype functions
void setupHaptics();
HapticsState fireHapticsEffect(uint8_t* newEffect);

void setupHaptics()
{
  drv.begin();
  
  drv.selectLibrary(1);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 

}
HapticsState fireHapticsEffect(uint8_t* newEffect)
{

  uint8_t dataLength = getLength(newEffect);
  Serial.printf("Data Length: %d\n", dataLength);
  if (dataLength <= 1 || dataLength > lastSlot) 
  {
    Serial.printf("Haptics Effect is empty with size %d", dataLength);
    return incomplete;
  }

  drv.setWaveform(0, newEffect[0]);
  drv.setWaveform(1, 0);

  // play the new effect!
  drv.go();

  // reset effect
  effect = {};
  return done;
}