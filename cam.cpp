#include "Arduino.h"
#include "cam.h"

cam::cam(HardwareSerial* ser) {
    _ser = ser;
}

void cam::begin() {
    _ser->begin(115200);
}


uint8_t cam::_crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

uint8_t cam::_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = _crc8_dvb_s2(crc, data[i]);
    }
    return crc;
}

void cam::_sendFrame(uint8_t cmd, uint8_t *data, uint8_t len) {
    uint8_t frame[128];
    frame[0] = 0xCC;
    frame[1] = cmd;
    memcpy(frame + 2, data, len);
    frame[len + 2] = _crc(frame, len + 2);

    _ser->write(frame, len + 3);
}


void cam::getInfo() {
    _sendFrame(COMMAND_GET_DEVICE_INFO, nullptr, 0);
}