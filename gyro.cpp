#include "Arduino.h"
#include "SPI.h"
#include "gyro.h"

gyro::gyro(SPIClass* SPI, SPISettings settings, int cs) {
    _SPI = SPI;
    _settings = settings;
    _cs = cs;
}

void gyro::begin() {
    _SPI->begin();
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, 1);
}

void gyro::_writeReg(byte reg, byte val) {
    digitalWrite(_cs, 0);
    _SPI->transfer(reg);
    _SPI->transfer(val);
    digitalWrite(_cs, 1);
}

void gyro::config() {
    _writeReg(0x6B, 0x81); //"To initialize the sensor, perform a reset and let the IAM-20380 select the best clock source by setting the register PWR_MGMT_1 (address 0x6B) to 0x81"
    delay(500);
    _writeReg(0x19, 0x03); //250 Hz
    _writeReg(0x1A, 0x03); //DPPF corner freq 41 Hz
    _writeReg(0x1B, 0x10); //1000 dps
}

int16_t gyro::gyro_x() {
    digitalWrite(_cs, 0);
    _SPI->transfer(READ | GYRO_XOUT_H);
    int16_t res = _SPI->transfer16(0x00);
    digitalWrite(_cs, 1);
    return res;
}

int16_t gyro::gyro_y() {
    digitalWrite(_cs, 0);
    _SPI->transfer(READ | GYRO_YOUT_H);
    int16_t res = _SPI->transfer16(0x00);
    digitalWrite(_cs, 1);
    return res;
}

int16_t gyro::gyro_z() {
    digitalWrite(_cs, 0);
    _SPI->transfer(READ | GYRO_ZOUT_H);
    int16_t res = _SPI->transfer16(0x00);
    digitalWrite(_cs, 1);
    return res;
}