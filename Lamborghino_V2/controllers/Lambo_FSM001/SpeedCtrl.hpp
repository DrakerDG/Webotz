#ifndef SPEED_CTRL_HPP
#define SPEED_CTRL_HPP

#include <webots/Supervisor.hpp>                  // Header of Webots Robot
#include <webots/Motor.hpp>                       // Header of Webots motor
#include "Config.hpp"                                       // Config header with configuration constants (thresholds, speeds)

class SpeedCtrl {
public:
  SpeedCtrl(webots::Supervisor *robot, int initSpeed);

  void setTarget(int target);                     // Function to set target speed
  void update();                                  // Function to update the current speed using the target speed
  void setSpeed(int leftSpeed, int rightSpeed);   // Module to update the speed of the motors.
  
  int  getSpeed() const;                          // Function to get the current speed
  int  getLimit() const;                          // Function to get the limit speed

private:
  webots::Motor *motor_L;                         // Webots motor_L
  webots::Motor *motor_R;                         // Webots motor_R
  
  int current_Speed;
  int  target_Speed;
  int   limit_Speed;
};

#endif

