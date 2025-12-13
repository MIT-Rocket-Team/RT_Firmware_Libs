#include "SPI.h"
#include "DRV8452.h"

#define MOSI PA7
#define MISO PA6
#define SCLK PA5
#define CS   PA4

bool direction = true;

SPIClass SPI_3(MOSI, MISO, SCLK, -1);
SPISettings settings(1000000, MSBFIRST, SPI_MODE0);

DRV8452 drv(&SPI_3, settings, CS);

void setup(){
  digitalWrite(_cs, 1)
  delay(1000);
  Serial.begin(115200);
  SPI_3.begin();
  drv.setup();
}

void loop(){
  for(float i = 0.0; i < 360; i += 1.8){
      drv.fullStep(direction);
      delayMicroseconds(20000);
  }

  direction = !direction;
}