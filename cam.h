//https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol
#include "Arduino.h"

#define COMMAND_GET_DEVICE_INFO 0x00

class cam {
    public:
        cam(HardwareSerial* ser);
        void begin();
        void getInfo();

    private:
        HardwareSerial* _ser;
        uint8_t _crc(uint8_t *data, uint8_t len);
        uint8_t _crc8_dvb_s2(uint8_t crc, unsigned char a);
        void _sendFrame(uint8_t cmd, uint8_t *data, uint8_t len);
};