#include "Arduino.h"
#include "SPI.h"
#include "ADXL357.h"

ADXL357::ADXL357(SPIClass* SPI, SPISettings settings, int cs) {
    _SPI = SPI;
    _settings = settings;
    _cs = cs;
}

void ADXL357::setup(){
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, 1);
    _writeReg(0x28, 0b00000011);
    _writeReg(0x2D, 0b00000000);
    _writeReg(0x2C, 0b10000011);
}

float ADXL357::getAccelX(){
    return _accelX;
}

float ADXL357::getAccelY(){
    return _accelY;
}

float ADXL357::getAccelZ(){
    return _accelZ;
}

uint8_t ADXL357::_readReg(byte reg) {
    digitalWrite(_cs, 0);
    _SPI->beginTransaction(_settings);
    _SPI->transfer((reg << 1) | 1);
    uint8_t data = _SPI->transfer(0x00);
    _SPI->endTransaction();
    digitalWrite(_cs, 1);
    return data;
}

bool ADXL357::_dataReady(){
    uint8_t data = _readReg(0x04);
    return data & 1;
}

void ADXL357::update() {
    if(_dataReady()){
        uint8_t data[10];
        data[0] = (0x08 << 1) | 1;
        digitalWrite(_cs, 0);
        _SPI->beginTransaction(_settings);
        _SPI->transfer(data, 10);
        _SPI->endTransaction();
        digitalWrite(_cs, 1);

        _accelXraw = ((data[1] << 16) + (data[2] << 8)+(data[3])) >> 4;
        _accelXraw = ((int32_t) (_accelXraw << 12)) >> 12;
        _accelX = _accelXraw / 12800.0 * 9.80665;
        _accelYraw = ((data[4] << 16) + (data[5] << 8)+(data[6])) >> 4;
        _accelYraw = ((int32_t) (_accelYraw << 12)) >> 12;
        _accelY = _accelYraw / 12800.0 * 9.80665;
        _accelZraw = ((data[7] << 16) + (data[8] << 8)+(data[9])) >> 4;
        _accelZraw = ((int32_t) (_accelZraw << 12)) >> 12;
        _accelZ = _accelZraw / 12800.0 * 9.80665;
    }
}

void ADXL357::_writeReg(byte reg, byte val) {
    _SPI->beginTransaction(_settings);
    digitalWrite(_cs, 0);
    _SPI->transfer(reg << 1);
    _SPI->transfer(val);
    digitalWrite(_cs, 1);
    _SPI->endTransaction();
}