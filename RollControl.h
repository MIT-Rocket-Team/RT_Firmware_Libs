#include "Arduino.h"
// #pragma once
// #include "stm32f7xx_hal.h"

class RollControl {
  public:

    enum Servo_State {
      SERVO_ZERO,
      SERVO_ANGLE,
      SERVO_PD
    };
    RollControl();
    void setup();
    void updateRollControl(Servo_State state, float angle, float velo);
    
  private:
    void updateITCallback();
    // static void staticUpdateITCallback();
    uint16_t degToUs(float degrees);
    void pidUpdate();
    void setServos(float degrees);
    uint16_t getServo6Us();
    uint16_t getServo7Us();
    // static void dispatch(TIM_HahndleTypeDef* htim);
    

    Servo_State _state;
    float _velo;
    uint16_t _servo6;
    uint16_t _servo7;
    static RollControl* instance;

    static void rollISR() {
      if (instance) {
        instance -> updateITCallback();
      }
    }  
    // TIM_HahndleTypeDef* _htim;
    // static RollControl* instance;
};
// RollControl* RollControl::instance = nullptr;