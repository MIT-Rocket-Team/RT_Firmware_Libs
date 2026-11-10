#include "Arduino.h"
#include "SINAT485.h"

SINAT485::SINAT485(HardwareSerial* incloSer){
  _incloSer = incloSer;
}

void SINAT485::begin(){
  _incloSer->begin(9600);
}