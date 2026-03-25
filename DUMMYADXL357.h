#include "Arduino.h"
#include "SPI.h"
#include "myTypes.h"

#define ACCEL_PREFLIGHT_INTEGRATION_THRESHOLD 10

class DUMMYADXL357 {
    public:
        DUMMYADXL357(SPIClass* SPI, SPISettings settings, int cs);
        void setup();
        float getAccelX();
        float getAccelY();
        float getAccelZ();
        int32_t getRawX();
        int32_t getRawY();
        int32_t getRawZ();
        void update(State rocketState, uint32_t simTime);
        float getVerticalAccelMinusGravity();
        float getIntegratedVelo();
        void zeroIntegratedVelo();
        
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        int _cs;
        float _accelX, _accelY, _accelZ;
        int32_t _accelXraw, _accelYraw, _accelZraw;
        void _writeReg(byte reg, byte val);
        bool _dataReady();
        uint8_t _readReg(byte reg);
        uint32_t _lastUpdate;
        float _integratedVelo;
        float _verticalAccelMinusGravity;
};