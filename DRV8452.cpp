void DRV8452::_setCurrentLimit(float current){
  byte value = (current * 0.66 / 3.3) * 256 - 1;
  _writeReg(0x0E, value);
  _writeReg(0x0D, value);
  _writeReg(0x10, 0x12);
}

void DRV8452::fullStep(bool forward){
  _writeReg(0x05, (forward << 7) | 0b01110000);
}

void DRV8452::setup(){
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