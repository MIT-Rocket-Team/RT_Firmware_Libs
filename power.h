#include "Arduino.h"
#include "myTypes.h"
#include <stdint.h>

class power {
  public:
    power(HardwareSerial* pwrSer);
    void begin();
    void update();
    uint16_t getConverterVoltage(uint8_t converter);
    uint16_t getConverterCurrent(uint8_t converter);
    uint16_t getCell1Voltage();
    uint16_t getCell2Voltage();
    uint16_t getCell3Voltage();
    uint16_t getTotalCurrent();
    uint8_t getProtectionStatus();
    float getTemp();

  private:
    HardwareSerial* _pwrSer;
    pwrBoardData _pkt;
    uint8_t _tmp[sizeof(_pkt)];
    uint8_t _calcChecksum(uint8_t* p);
};