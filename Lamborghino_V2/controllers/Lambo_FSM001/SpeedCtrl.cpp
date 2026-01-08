#include "SpeedCtrl.hpp"

using namespace webots;

SpeedCtrl::SpeedCtrl(Supervisor *robot, int initSpeed) {
  motor_L = robot->getMotor("motor_L");                     // Get device (motor_L)
  motor_R = robot->getMotor("motor_R");                     // get device (motor_R)
  motor_L->setPosition(INFINITY);                           // Set position to continuous motor (motor_L)
  motor_R->setPosition(INFINITY);                           // Set position to continuous motor (motor_R)
  motor_L->setVelocity(initSpeed);                          // Set velocity 0 rad/s (motor_L)
  motor_R->setVelocity(initSpeed);                          // Set velocity 0 rad/s (motor_R)    

  current_Speed = initSpeed;
   target_Speed = initSpeed;
}

void SpeedCtrl::setTarget(int target) {                     // Function to set target speed
  target_Speed = target;
  switch (target) {
    case SP001: limit_Speed = SP002; break;
    case SP003: limit_Speed = SP004; break;
    case SP005: limit_Speed = SP006; break;
    default:    limit_Speed = SP002;
  }

}

void SpeedCtrl::update() {                                  // Function to update the current speed using the target speed
  if (current_Speed < target_Speed) {
    current_Speed += ST_CH;
    if (current_Speed > target_Speed)
      current_Speed = target_Speed;
  }
  else if (current_Speed > target_Speed) {
    current_Speed -= ST_CH;
    if (current_Speed < target_Speed)
      current_Speed = target_Speed;
  }
}

void SpeedCtrl::setSpeed(int  leftSpeed, 
                         int rightSpeed) {
  motor_L->setVelocity( leftSpeed);                         // Set the calibration speed (motor_L)
  motor_R->setVelocity(rightSpeed);                         // Set the calibration speed (motor_R)
}

int SpeedCtrl::getSpeed() const {                           // Function to get the current speed
  return current_Speed;
}

int SpeedCtrl::getLimit() const {                           // Function to get the limit speed
  return limit_Speed;
}

