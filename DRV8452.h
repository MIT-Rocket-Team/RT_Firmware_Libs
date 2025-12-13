#include "Arduino.h"
#include "SPI.h"

class DRV8452 {
    public:
        DRV8452(SPIClass* SPI, SPISettings settings, int cs);
        void fullStep(bool foward);
        void setup();
        
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        int _cs;
        void _writeReg(byte reg, byte val);
        void _setCurrentLimit(float current);
};