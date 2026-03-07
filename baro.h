#include "Arduino.h"
#include "SPI.h"

#define C1 0xA579
#define C2 0x953A
#define C3 0x68AC
#define C4 0x6305
#define C5 0x8405
#define C6 0x6D91


class baro {
    public:
        baro(SPIClass* SPI, SPISettings settings, int cs);
        void begin();
        void updateRawTemp();
        void updateRawPress();
        uint32_t getRawTemp();
        uint32_t getRawPress();
        uint16_t getCalibrationConstant(uint8_t index);
        float getTemperature();
        float getPressure();
        float getAltitude();
        
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        int _cs;
        uint32_t _rawTemp, _rawPress;
};