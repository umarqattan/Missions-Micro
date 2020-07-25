// MARK: - Includes
#include "Adafruit_DRV2605.h"


// MARK: - Haptics Driver variables
Adafruit_DRV2605 drv;
uint8_t* effect;
int hapticsSampleRate = 500;
uint8_t lastSlot = 7;
uint8_t lastEffect = 117;

void setupHaptics()
{
  drv.begin();
  
  drv.selectLibrary(1);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 

}