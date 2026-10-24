#include "Arduino.h"
#include "SPI.h"

class ADS1220 {
    public:
        ADS1220(SPIClass* SPI, SPISettings settings, int cs);
        void setup();
        void update();
        uint32_t getMeasurement();
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        uint32_t _measurement;
        int _cs;
};