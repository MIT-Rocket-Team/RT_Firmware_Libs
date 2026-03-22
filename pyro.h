#include "Arduino.h"
#include "myTypes.h"

#define PYRO_FIRE_DURATION 250

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
        uint16_t getPyrosStatus();
        PyroStatus getPyroStatus(uint8_t channel);
        void update(State rocketState);
        void _setPyroStatus(uint8_t channel, PyroStatus val);

    private:
        bool _armed[6];
        bool _fired[6];
        uint32_t _firedTimes[6];
        uint16_t _pyroStatus;
};
