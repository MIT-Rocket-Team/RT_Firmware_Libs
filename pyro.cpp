#include "pyro.h"

int adca[6] = {PB1, PB0, PC5, PC4, PA7, PA6};
int adcb[6] = {PC0, PC1, PA0, PA1, PA2, PA3};
int firePins[6] = {PB3, PB5, PB6, PB7, PB8, PB9};
int sens[6] = {PE15, PE14, PE13, PE12, PE11, PE10};


pyro::pyro() {
}

void pyro::begin() {    
    for (int i = 0; i < 6; i++) {
        pinMode(firePins[i], OUTPUT);
        pinMode(sens[i], INPUT);
    }
}

bool pyro::connected(uint8_t channel) {
    return digitalRead(sens[channel]);
}

float pyro::resistance(uint8_t channel) {
    return 0.5 * (((float) analogRead(adcb[channel])) / ((float)analogRead(adca[channel])));
}

void pyro::arm(uint8_t channel) {
    _armed[channel] = true;
}

void pyro::fire(uint8_t channel) {
    if (_armed[channel]) {
        digitalWrite(firePins[channel], HIGH);
        _firedTimes[channel] = millis();
        _fired[channel] = true;
        _armed[channel] = false;
    }
}

void pyro::off(uint8_t channel) {
    digitalWrite(firePins[channel], LOW);
    _fired[channel] = false;
}

bool pyro::isArmed(uint8_t channel) {
    return _armed[channel];
}

bool pyro::isFired(uint8_t channel) {
    return _fired[channel];
}

uint16_t pyro::getPyrosStatus() {
    return _pyroStatus;
}

PyroStatus pyro::getPyroStatus(uint8_t channel) {
  return static_cast<PyroStatus> ((uint8_t) ((_pyroStatus >> (channel * 2)) & 0b11));
}

void pyro::update(State rocketState) {
    for (uint8_t i = 0; i < 6; i++) {
        //turn off pyros if they have been on for > PYRO_FIRE_DURATION
        if (_fired[i] && millis() - _firedTimes[i] > PYRO_FIRE_DURATION) {
            off(i);
            //If pyro no longer connected and didn't come unconnected earlier, then it's a success
            if (!connected(i) && getPyroStatus(i) != PYRO_FAILURE) {
                _setPyroStatus(i, PYRO_SUCCESS);
            } else {
                _setPyroStatus(i, PYRO_FAILURE);
            }
        }
        //Update pyro state
        //in ground testing, we only care if pyro is connected or disconnected
        if (rocketState == GROUND_TESTING) {
            if (connected(i)) {
                _setPyroStatus(i, PYRO_CONNECTED);
            } else {
                _setPyroStatus(i, PYRO_UNCONNECTED);
            }
        } else {
            //Otherwise, an unconnected pyro is a failure unless it has been successfully fired
            if (!connected(i) && getPyroStatus(i) != PYRO_SUCCESS && !_fired[i]) {
                _setPyroStatus(i, PYRO_FAILURE);
            }
        }
    }
    
}

void pyro::_setPyroStatus(uint8_t channel, PyroStatus val) {
  uint16_t mask = 0b11 << (channel * 2);
  _pyroStatus &= ~mask;
  _pyroStatus |= (val & 0b11) << (channel * 2);
}