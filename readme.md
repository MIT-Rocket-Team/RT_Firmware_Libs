# **RT Firmware Libs**

## Structure
Arduino code is at the top level. For other files, see the directory `.other`, in which the following can be found:

- `docs/system_architecture`: documentation regarding the structure of the code
- `src/java.sim.nocompile/*`: java code for the OR simulator at (`MDNich/ActiveControl_MIT_RktTeam`)[https://github.com/MDNich/ActiveControl_MIT_RktTeam]. It will not compile in this repository, and it is purely for reference purposes.


## **GPS library**

### `GPS(HardwareSerial* gpsSer)`
constructor
### `void setup()`
begins serial port
### `void updateAndParse()`
moves any available serial data into the data buffer, and if any data is available, parses the buffer
### `float getPDOP()`
returns PDOP
### `float getVDOP()`
returns VDOP
### `float getHDOP()`
returns HDOP
### `float getLatitude()`
returns latitude
### `float getLongitude()`
returns longitude
### `float getAltitude()`
returns altitude in meters
### `bool getFix()`
returns true if GPS fix; false otherwise