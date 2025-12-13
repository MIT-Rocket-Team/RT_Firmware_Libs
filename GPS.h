/*
GPS: GPS setup and parsing
*/
#include "Arduino.h"

class GPS {
  public:
    GPS(HardwareSerial* gpsSer, float gpsAltOffset);
    GPS(USBSerial* gpsSer, float gpsAltOffset);
    void setup();
    void updateAndParse();
    float getPDOP();
    float getVDOP();
    float getHDOP();
    float getLatitude();
    float getLongitude();
    float getAltitude();
    bool getFix();
  private:
    Stream* _gpsSer;
    HardwareSerial* _hwGpsSer;
    USBSerial* _usbGpsSer;
    bool hwSerialUsed;
    float _pdop;
    float _vdop;
    float _hdop;
    float _latitude;
    float _longitude;
    float _altitude;
    bool _fix;
    char _gpsData[100];
    float _rawGPS;
    float _gpsAltOffset;
    float _maxGpsAlt;

    bool _gpsUpdateData(int length);
    void _gpsParse(char *data, int length);
    float _convertToDecimalFast(float raw, char hemi);
    int _indexOf(const char* arr, char target, int start);
};