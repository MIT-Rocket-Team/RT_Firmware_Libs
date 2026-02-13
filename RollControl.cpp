#include "Arduino.h"
#include "RollControl.h"

#define kP 0.1887
#define kD 0.03774
#define setpoint 0.0
#define OFFSET1 7 //CHANNEL 1 (PA8, HEADER 7, RED WIRE)
#define OFFSET2 -2 //CHANNEL 2 (PA9, HEADER 6, ALL BLACK WIRES)
RollControl* RollControl::instance = nullptr;

struct downlink {
  //MAG
  int32_t rawMagX;
  int32_t rawMagY;
  int32_t rawMagZ;
  float heading;
  //GYRO
  int16_t rawGyroX;
  int16_t rawGyroY;
  int16_t rawGyroZ;
  float dpsX;
  float dpsY;
  float dpsZ;
  float yaw;
  float pitch;
  float roll;
  //SERVO
  uint16_t us_on_6 = 1000;
  uint16_t us_on_7 = 1050;
};

downlink pkt;
HardwareTimer *myTim = new HardwareTimer(TIM1);

/**
 * Initializes state to zero.
 */
RollControl::RollControl(){
    _state = SERVO_ZERO;
}

/**
 * Start hardware timers to servo zero point
 */
void RollControl::setup(){
    myTim->setMode(1, TIMER_OUTPUT_COMPARE_PWM1, PA_8);
    myTim->setMode(2, TIMER_OUTPUT_COMPARE_PWM1, PA9);
    myTim->setOverflow(20000, MICROSEC_FORMAT);
    myTim->setCaptureCompare(1, degToUs(OFFSET1), MICROSEC_COMPARE_FORMAT);
    myTim->setCaptureCompare(2, degToUs(OFFSET2), MICROSEC_COMPARE_FORMAT);
    myTim->attachInterrupt(rollISR);
    myTim->resume();
}

/**
 * Run pidUpdate() if state is at `SERVO_PD`
 */
void RollControl::updateITCallback() {
  if (_state == SERVO_PD) {
    pidUpdate();
  }
}

/**
 * Converts degrees to pulse width
 */
uint16_t RollControl::degToUs(float degrees) {
  return 1500.0 + (degrees / 60.0) * 500.0; 
}

/**
 * Update fin states and measurements
 * @param state - desired fin state
 * @param angle - degrees
 * @param velo - velocity, m/s
 */
void RollControl::updateRollControl(Servo_State state, float angle, float velo){
    _state = state;
    _velo = velo;

    switch (_state) {
        case SERVO_ZERO:
            setServos(0.0);
            break;
        case SERVO_ANGLE:
            setServos(angle);
            break;
  }

}

/**
 * Set servos to degree
 */
void RollControl::setServos(float degrees) {
  myTim->setCaptureCompare(1, degToUs(degrees + OFFSET1), MICROSEC_COMPARE_FORMAT);
  myTim->setCaptureCompare(2, degToUs(degrees + OFFSET2), MICROSEC_COMPARE_FORMAT);
  _servo6 = degToUs(degrees + OFFSET1);
  _servo7 = degToUs(degrees + OFFSET2);
}

/**
 * Update PID
 * `kP`, `kD`, `velo` (m/s), `setpoint` (deg), `roll` (deg), `dpsY` (deg/s) are global vars (type float) that get updated elsewhere.
   Note we use dpsY because the y axis on the gyro is alligned with the roll axis of rocket.
 */
void RollControl::pidUpdate(){
      /*
  kP, kD, velocity (m/s), setpoint (deg), roll (deg), dpsY (deg/s) are global vars (type float) that get updated elsewhere.
  Note we use dpsY because the y axis on the gyro is alligned with the roll axis of rocket.
  */
  float velocity_pid = (_velo < 20.0) ? 20 : _velo;
  //SAFEGUARD: Protect against absurdly high velocitites
  // float velocity_pid = (_velo > 300.0) ? 300 : _velo;
  float scaled_kP = kP * 10000 / pow(velocity_pid, 2);
  float scaled_kD = kD * 10000 / pow(velocity_pid, 2);
  /*
  Roll is positive CW, and a positive angle on the sevos rolls the rocket CW.
  Therefore, we don't need to negate PD signs.
  PD control output = kP * e(t) + kD * de/dt
  e(t) = setpoint - measured = setpoint - roll
  de/dt = d(setpoint - roll)/dt = -dpsY
  As a sanity check, rocket at roll = 10 deg and zero angular velo,
  then pdOutput will be negative, so servos will drive CCW, causing rocket to roll CCW.
  */
  //FIX: DO NOT SCALE scaled_kD by PI/180
  float pdOutput = scaled_kP * (setpoint - pkt.roll) + scaled_kD * -pkt.dpsY;

  //Clip to -10 and 10 deg
  if (pdOutput > 10) {
    pdOutput = 10.0;
  } else if (pdOutput < -10) {
    pdOutput = -10.0;
  }
  setServos(pdOutput);
}

/**
 * Get pulse width for servo6 (channel 1)
 */
uint16_t RollControl::getServo6Us(){
    return _servo6;
}

/**
 * Get pulse width for servo7 (channel 2)
 */
uint16_t RollControl::getServo7Us(){
    return _servo7;
}