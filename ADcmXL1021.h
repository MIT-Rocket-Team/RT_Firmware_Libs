#ifndef ADCMXL1021_H
#define ADCMXL1021_H

#include "Arduino.h"
#include "SPI.h"

class ADcmXL1021 {
public:
    static constexpr uint8_t PRODUCT_ID_REGISTER = 0x56;
    static constexpr uint16_t EXPECTED_PRODUCT_ID = 0x03FD;

    ADcmXL1021(SPIClass* spi, SPISettings settings, int chipSelect);
    void begin();
    uint16_t readRegister(uint8_t lowerByteAddress);
    uint16_t readProductId();

private:
    SPIClass* _spi;
    SPISettings _settings;
    int _chipSelect;
};

#endif
