#ifndef MY_TYPES_H
#define MY_TYPES_H

enum State {
  GROUND_TESTING,
  PRE_FLIGHT,
  FLIGHT,
  APOGEE,
  DISREEF,
  END
};

typedef struct __attribute__((packed))
{
  int16_t cell1;
  int16_t cell2;
  int16_t cell3;
  int16_t stackVoltage;
  int16_t current;
  float temp;
  uint8_t fetStatus;
} bmsData;

typedef struct __attribute__((packed))
{
  bmsData BMS;
  int16_t voltages[6];
  int16_t currents[6];
} pwrBoardData;

#endif