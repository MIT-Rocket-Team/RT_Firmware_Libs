#include "Arduino.h"
#include "SPI.h"
#include "DRV8452.h"

DRV8452::DRV8452(SPIClass* SPI, SPISettings settings, int cs, int slp, int en) {
    _SPI = SPI;
    _settings = settings;
    _cs = cs;
    _slp = slp;
    _en = en;
}

void DRV8452::setCurrentLimit(float current){
  uint8_t value = (current * 0.66 / 3.3) * 256 - 1;
  _writeReg(0x0E, value);
  _writeReg(0x0D, value);
  _writeReg(0x10, 0x12);
}

float DRV8452::getStepCurrentLimit() {
  uint8_t value = _readReg(0x0E);
  return ((value + 1) / 256.0) * (3.3 / 0.66);
}

float DRV8452::getHoldCurrentLimit() {
  uint8_t value = _readReg(0x0D);
  return ((value + 1) / 256.0) * (3.3 / 0.66);
}

void DRV8452::fullStep(bool forward){
  _writeReg(0x05, (forward << 7) | 0b01110000);
}

void DRV8452::setup(){
  pinMode(_cs, OUTPUT);
  pinMode(_slp, OUTPUT);
  pinMode(_en, OUTPUT);
  digitalWrite(_cs, 1);
  digitalWrite(_slp, 1);
  digitalWrite(_en, 1);
  setCurrentLimit(3.5);
  _writeReg(0x05, 0b00110000);
}

void DRV8452::_writeReg(byte reg, byte val) {
  _SPI->beginTransaction(_settings);
  digitalWrite(_cs, 0);
  _SPI->transfer(reg);
  _SPI->transfer(val);
  digitalWrite(_cs, 1);
  _SPI->endTransaction();
  delayMicroseconds(1);
}

uint8_t DRV8452::_readReg(byte reg) {
  _SPI->beginTransaction(_settings);
  digitalWrite(_cs, 0);
  _SPI->transfer(reg | 0x40);
  uint8_t val = _SPI->transfer(0x00);
  digitalWrite(_cs, 1);
  _SPI->endTransaction();
  delayMicroseconds(1);
  return val;
}