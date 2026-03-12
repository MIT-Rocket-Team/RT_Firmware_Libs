#include "Arduino.h"
#include "GPS.h"

uint8_t GPS_10HZ[14] = {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12};
uint8_t GPS_UBX_ENABLE[16] = {0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x18,0xE1};
uint8_t GPS_SERIAL_CONFIG[28] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0xC2,0x01,0x00,0x07,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0xBE,0x72};
uint8_t GPS_CONFIG_UPDATE[9] = {0xB5,0x62,0x06,0x00,0x01,0x00,0x01,0x08,0x22};

GPS::GPS(HardwareSerial* gpsSer){
  _gpsSer = gpsSer;
}

void GPS::begin() {
  _gpsSer.begin(9600);
  _gpsSer.write(GPS_10HZ, 14);
  _gpsSer.write(GPS_UBX_ENABLE, 16);
  _gpsSer.write(GPS_SERIAL_CONFIG, 28);
  _gpsSer.write(GPS_CONFIG_UPDATE, 9);
  _gpsSer.flush();
  _gpsSer.begin(115200);
}

void GPS::update() {
  while (_gpsSer.available()) {
    if (_validateHeader()) {
        if(_gpsSer.available() >= 98) {
            _readPacket();
            if (_validateChecksum()) {
              memcpy(&_pkt, _buf + 4, 92);
            }
        }
    }
  }
}

bool GPS::_validateHeader() {
  return GPS.read() == 0xB5 && GPS.read() == 0x62 && GPS.read() == 0x01 && GPS.read() == 0x07 && GPS.read() == 0x5C && GPS.read() == 0x00;
}

void GPS::_readPacket() {
  _gpsSer.readBytes(_buf + 4, 98);
  _buf[0] = 0x01;
  _buf[1] = 0x07;
  _buf[2] = 0x5C;
  _buf[3] = 0x00;
}

void GPS::_validateChecksum() {
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 0; i < 96; i++) {
    ck_a += _buf[i];
    ck_b += ck_a;
  }
  return ck_a == _buf[96] && ck_b == _buf[97];
}