#include "Arduino.h"

class SINAT485 {
  public:
    SINAT485(HardwareSerial* incloSer);
    void begin();

  private:
    HardwareSerial* _incloSer;
};