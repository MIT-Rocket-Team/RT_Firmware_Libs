#include "Arduino.h"
#include "GPS.h"

GPS::GPS(HardwareSerial* gpsSer, float gpsAltOffset){
  _gpsSer = gpsSer;
  _gpsAltOffset = gpsAltOffset;
}

void GPS::setup(){
  _gpsSer -> begin(115200);
}

void GPS::updateAndParse(){
  _gpsUpdateData(sizeof(_gpsData)); //sizeOf() if this doesn't work, replace with 100
  _gpsParse(_gpsData, sizeof(_gpsData));
}

float GPS::getPDOP(){
  return _pdop;
}

float GPS::getVDOP(){
  return _vdop;
}

float GPS::getHDOP(){
  return _hdop;
}

float GPS::getLatitude(){
  return _latitude;
}

float GPS::getLongitude(){
  return _longitude;
}

float GPS::getAltitude(){
  return _altitude;
}

bool GPS::getFix(){
  return _fix;
}

///////// PRIVATE METHODS //////////

/**
Finds index of target in char array
@param start starting index, inclusive
@return int, -1 if not found
*/
int GPS::_indexOf(const char* arr, char target, int start) {
  for (int i = start; arr[i] != '\0'; i++) {
    if (arr[i] == target) {
      return i;   // found
    }
  }
  return -1;      // not found
}

/**
Converts degrees to decimals
*/
float GPS::_convertToDecimalFast(float raw, char hemi) {
    int deg = raw / 100;
    float minutes = raw - deg * 100;
    float decimal = deg + minutes / 60.0;

    if (hemi == 'S' || hemi == 'W')
        decimal = -decimal;

    return decimal;
}

/**
Updates GPS data buffer
@param length length of buffer
*/
bool GPS::_gpsUpdateData(int length){
  uint16_t avail = _gpsSer -> available();
  while (avail > length) {
    _gpsSer -> read();
    avail--;
  }
  if (avail) {
    //Shift existing data to the left
    for(uint8_t i = avail; i < length; i++) {
      _gpsData[i - avail] = _gpsData[i];
    }
    //Read new data into the end
    for (uint8_t i = avail; i > 0; i--) {
      _gpsData[length - i] = _gpsSer -> read();
    }
    return true;
  }
  return false;
}

/**
Parse GPS data 
@param length length of the buffer
*/
void GPS::_gpsParse(char *data, int length) {
    int start = 0;
    while (start < length) {
        int newStart = start != 0 ? start + 10: start;
        int end = _indexOf(data, '\n', newStart);
        if (end == -1) end = length;
        char *line = data + start;
        // ---------- FAST CHECKSUM ----------
        char *star = (char*)memchr(line, '*', end - start);
        if (!star) { start = end + 1; continue; }

        uint8_t sum = 0;
        for (char *p = line + 1; p < star; p++) sum ^= *p;

        uint8_t transmitted = strtol(star + 1, NULL, 16);
        if (sum != transmitted) {
            start = end + 1;
            continue;
        }

        // ---------- PARSE SENTENCES ----------
        if (memcmp(line, "$GNRMC", 6) == 0) {

            // Fields by pointer
            char *fields[12] = {0};
            int fieldIndex = 0;
            fields[0] = line;

            for (char *p = line; p < line + (end - start); p++) {
                if (*p == ',') {
                    *p = '\0';
                    fields[++fieldIndex] = p + 1;
                }
            }

            _fix = (fields[2][0] == 'A');

            float rawLat = atof(fields[3]);
            float rawLon = atof(fields[5]);

            char hemiLat = fields[4][0];
            char hemiLon = fields[6][0];

            _latitude = convertToDecimalFast(rawLat, hemiLat);
            _longitude = convertToDecimalFast(rawLon, hemiLon);
        }

        else if (memcmp(line, "$GNGGA", 6) == 0) {

            char *fields[15] = {0};
            int idx = 0;
            fields[0] = line;

            for (char *p = line; p < line + (end - start); p++) {
                if (*p == ',') {
                    *p = '\0';
                    fields[++idx] = p + 1;
                }
            }

            float alt = atof(fields[9]);
            float geoid = atof(fields[11]);

            _rawGPS = alt + geoid;
            _altitude = _rawGPS - _gpsAltOffset;

            if (_fix && _altitude > _maxGpsAlt)
                _maxGpsAlt = _altitude;
        }

        else if (memcmp(line, "$GNGSA", 6) == 0) {

            char *fields[25] = {0};
            int idx = 0;
            fields[0] = line;

            for (char *p = line; p < line + (end - start); p++) {
                if (*p == ',') {
                    *p = '\0';
                    fields[++idx] = p + 1;
                }
            }

            _pdop = atof(fields[15]);
            _hdop = atof(fields[16]);
            _vdop = atof(fields[17]);
        }

        start = end + 1;
    }
}
