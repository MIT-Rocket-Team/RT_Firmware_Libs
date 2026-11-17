#include "ADcmXL1021.h"

ADcmXL1021::ADcmXL1021(SPIClass* spi, SPISettings settings, int chipSelect)
    : _spi(spi), _settings(settings), _chipSelect(chipSelect) {}

void ADcmXL1021::begin() {
    pinMode(_chipSelect, OUTPUT);
    digitalWrite(_chipSelect, HIGH);
    _spi->begin();

    // ADcmXL1021-1 startup time is specified as 205 ms after VDD reaches 3.0 V.
    delay(250);
}

uint16_t ADcmXL1021::readRegister(uint8_t lowerByteAddress) {
    _spi->beginTransaction(_settings);
    digitalWrite(_chipSelect, LOW);

    delayMicroseconds(1);

    // The ADcmXL interface uses 16-clock words. Send the read/address word
    // explicitly as bytes: R/W = 0, followed by the 7-bit register address.
    uint8_t address = lowerByteAddress & 0x7F;
    _spi->transfer16((uint16_t)(lowerByteAddress & 0x7F) << 8);
    digitalWrite(_chipSelect, HIGH);

    delayMicroseconds(20);
    
    digitalWrite(_chipSelect, LOW);
    uint16_t response = _spi->transfer16(0x0000);

    digitalWrite(_chipSelect, HIGH);
    _spi->endTransaction();

    // The register readout is little-endian; return a host-order 16-bit word.
    return ((uint16_t)response >> 8) | (response << 8);
}
uint16_t ADcmXL1021::readProductId() {
    return readRegister(PRODUCT_ID_REGISTER);
}
