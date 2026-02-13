#include <RollControl.h>
//COMMENT THIS TO USE HARDWARE SERIAL
#define USE_USB_SER

#ifdef USE_USB_SER
#define GPS_SER Serial
#else
HardwareSerial GPS_SER(PA1, PA0);
#endif

RollControl rollControl;

unsigned long last;

void setup() {
  // put your setup code here, to run once:
  rollControl.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
}
