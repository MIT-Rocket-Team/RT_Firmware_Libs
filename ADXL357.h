#include "Arduino.h"
#include "SPI.h"

class ADXL357 {
    public:
        ADXL357(SPIClass* SPI, SPISettings settings, int cs);
        void setup();
        int32_t getAccelX();
        int32_t getAccelY();
        int32_t getAccelZ();
        void update();
        
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        int _cs;
        int32_t _accelX, _accelY, _accelZ;
        void _writeReg(byte reg, byte val);
        bool _dataReady();
        uint8_t _readReg(byte reg);
};