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