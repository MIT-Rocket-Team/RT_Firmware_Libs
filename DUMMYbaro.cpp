#include "Arduino.h"
#include "SPI.h"
#include "DUMMYbaro.h"

//_filteredAlt
static float dummyData[10000];

float dT, TEMP, P;
int64_t OFF, SENS;
const float P0 = 1013.25;

DUMMYbaro::DUMMYbaro(SPIClass* SPI, SPISettings settings, int cs) {
    _SPI = SPI;
    _settings = settings;
    _cs = cs;
}

void DUMMYbaro::begin(){
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, 1);
}

void DUMMYbaro::updateRawPress() {
    digitalWrite(_cs, 0);
    _SPI->beginTransaction(_settings);
    _SPI->transfer(0x42);
    _SPI->endTransaction();
    digitalWrite(_cs, 1);

    delayMicroseconds(1500);

    digitalWrite(_cs, 0);
    _SPI->beginTransaction(_settings);
    _SPI->transfer(0x00);
    _rawPress  = _SPI->transfer16(0x00) << 8;
    _rawPress += _SPI->transfer(0x00);
    _SPI->endTransaction();
    digitalWrite(_cs, 1);
}

void DUMMYbaro::updateRawTemp() {
    digitalWrite(_cs, 0);
    _SPI->beginTransaction(_settings);
    _SPI->transfer(0x52);
    _SPI->endTransaction();
    digitalWrite(_cs, 1);

    delayMicroseconds(1500);

    digitalWrite(_cs, 0);
    _SPI->beginTransaction(_settings);
    _SPI->transfer(0x00);
    _rawTemp  = _SPI->transfer16(0x00) << 8;
    _rawTemp += _SPI->transfer(0x00);
    _SPI->endTransaction();
    digitalWrite(_cs, 1);
}

uint32_t DUMMYbaro::getRawTemp() {
    return _rawTemp;
}

uint32_t DUMMYbaro::getRawPress() {
    return _rawPress;
}

float DUMMYbaro::getTemperature(){
    dT = (float)_rawTemp - ((float)C5)*((int)1<<8);
    TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
    return TEMP/100;
}

float DUMMYbaro::getPressure() {
    dT = (float)_rawTemp - ((float)C5)*((int)1<<8);
    TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
    OFF = (((int64_t)C2)*((long)1<<17)) + dT * ((float)C4)/((int)1<<6);
    SENS = ((float)C1)*((long)1<<16) + dT * ((float)C3)/((int)1<<7);
    float pa = (float)((float)_rawPress/((long)1<<15));
    float pb = (float)(SENS/((float)((long)1<<21)));
    float pc = pa*pb;
    float pd = (float)(OFF/((float)((long)1<<15)));
    P = pc - pd;
    return P/100;
}

float DUMMYbaro::getAltitude() {
    float h,t,p;
    t = getTemperature();
    p = getPressure();
    p = P0/p;
    h = 153.84615*(pow(p,0.19) - 1)*(t+273.15);
    return h;
}

uint16_t DUMMYbaro::getCalibrationConstant(uint8_t index) {
    digitalWrite(_cs, 0);
    _SPI->beginTransaction(_settings);
    _SPI->transfer(0b10100000 | (index << 1));
    uint16_t res = _SPI->transfer16(0x00);
    _SPI->endTransaction();
    digitalWrite(_cs, 1);
    return res;
}

float DUMMYbaro::getFilteredAltitude() {
    return _filteredAlt - _heightOffset;
}

void DUMMYbaro::updateAll(uint32_t simTime) {
    updateRawTemp();
    updateRawPress();
    for(int8_t i = MAVG_SAMPLES - 2; i > - 1; i--) {
        _filteredAltSamples[i+1] = _filteredAltSamples[i];
    }
    _filteredAltSamples[0] = getAltitude();
    _filteredAlt = 0;
    for (uint8_t i = 0; i < 20; i++) {
        _filteredAlt += _filteredAltSamples[i];
    }
    _filteredAlt /= MAVG_SAMPLES;

    _filteredAlt = dummyData[simTime / 10];

    if (getFilteredAltitude() > _maxAlt) {
        _maxAlt = getFilteredAltitude();
    }
}

float DUMMYbaro::getMaxAlt() {
    return _maxAlt;
}

void DUMMYbaro::zeroAlt() {
    _heightOffset = getFilteredAltitude();
    _maxAlt = 0;
}