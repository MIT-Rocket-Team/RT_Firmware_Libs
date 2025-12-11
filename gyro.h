#include "Arduino.h"
#include "SPI.h"

#define READ 0x80

#define GYRO_XOUT_L 0x44
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_L 0x46
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_L 0x48
#define GYRO_ZOUT_H 0x47

class gyro {
    public:
        gyro(SPIClass* SPI, SPISettings settings, int cs);
        void begin();
        void config();
        int16_t gyro_x();
        int16_t gyro_y();
        int16_t gyro_z();
        
    private:
        SPIClass* _SPI;
        SPISettings _settings;
        int _cs;
        void _writeReg(byte reg, byte val);
};