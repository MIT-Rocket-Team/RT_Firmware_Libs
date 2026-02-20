#include "Arduino.h"
#include "SPI.h"

class ADXL357 {
    public:
        ADXL357(SPIClass* SPI, SPISettings settings, int cs);
        void setup();
        float getAccelX();
        float getAccelY();
        float getAccelZ();
        void update();
        
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        int _cs;
        float _accelX, _accelY, _accelZ;
        int32_t _accelXraw, _accelYraw, _accelZraw;
        void _writeReg(byte reg, byte val);
        bool _dataReady();
        uint8_t _readReg(byte reg);
};