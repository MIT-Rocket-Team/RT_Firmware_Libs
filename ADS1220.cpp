#include "Arduino.h"
#include "SPI.h"
#include "ADS1220.h"

ADS1220::ADS1220(SPIClass* SPI, SPISettings settings, int cs) {
    _SPI = SPI;
    _settings = settings;
    _cs = cs;
}

void ADS1220::setup() {
    //Set cs pin as output and set it high
    
    //Begin _SPI, and run beginTransaction with _settings

    //Set AINP = AIN1, AINN = AIN2, gain = 128, PGA enabled (Configuration Register 0)
    
    //Set the ADS1220 in continuous conversion mode in normal mode at 175SPS (Configuration Register 1)
    
    //Set the ADS1220 to use "External reference selected using AIN0/REFP1 and AIN3/REFN1 inputs" as the voltage reference selection (Configuration Register 2)
    //Also, in Configuration Register 2, set Low-side power switch configuration to "Switch automatically closes when the START/SYNC command is sent and
    //opens when the POWERDOWN command is issued"
    
    //Send the start command to the ADS1220 to begin conversions
}

void ADS1220::update() {
    //Send the RDATA command to the ADS1220 to read the conversion result

    //Read the 3 bytes of data from the ADS1220 and store it in _measurement
}

uint32_t ADS1220::getMeasurement() {
    return _measurement;
}