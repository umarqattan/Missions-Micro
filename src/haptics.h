// MARK: - Includes
#include "Adafruit_DRV2605.h"


void setupHaptics();
void hapticsLoop();

// MARK: - Haptics Driver variables
Adafruit_DRV2605 drv;
uint8_t effect = 1;
int hapticsSampleRate = 500;


void setupHaptics()
{
  drv.begin();
  
  drv.selectLibrary(1);
  
  // I2C trigger by sending 'go' command 
  // default, internal trigger when sending GO command
  drv.setMode(DRV2605_MODE_INTTRIG); 

}

void hapticsLoop()
{
  
  // set the effect to play
  drv.setWaveform(0, effect);  // play effect 
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();

  // wait a bit
  delay(hapticsSampleRate);

  effect++;
  if (effect > 117) effect = 1;
}