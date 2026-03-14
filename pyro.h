#include "Arduino.h"

class pyro {
    public:
        pyro();
        void begin();
        bool connected(uint8_t channel);
        float resistance(uint8_t channel);
        void arm(uint8_t channel);
        void fire(uint8_t channel);
        void off(uint8_t channel);
        bool isArmed(uint8_t channel);
        bool isFired(uint8_t channel);
    private:
        bool _armed[6];
        bool _fired[6];
};
