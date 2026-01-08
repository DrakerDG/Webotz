// File:          Follow_Cam001.cpp |  Main  |
// Date:          January 2026
// Description:   Controller in C++ to follower camera
// Author:        DrakerDG
// Version:       2.0
//        ____             __             ____  ______
//       / __ \_________ _/ /_____  _____/ __ \/ ____/
//      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
//     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
//    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
//                                             

#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Motor.hpp>           // Header of Webots Motor
#include <cmath>

using namespace webots;

int main(int argc, char **argv) {

  Supervisor *robot = new Supervisor();                   // create the Robot instance
  
  int timeStep = (int)robot->getBasicTimeStep();          // get the time step of the current world

  Node *robotNode = robot->getFromDef("Lamborghino");           // Get the node by DEF or robot
  Field *robotPos = robotNode->getField("translation");   // Get the "translation" field
 
  Node *camNode = robot->getFromDef("Follower_cam");      // Get the node by DEF of camera
  Field *camPos = camNode->getField("translation");       // Get the "translation" field

  double current_pos[3];
  
  // Initialize motors
  Motor *motor_cam = robot->getMotor("motor::pan");       // Get device (motor_cam)
  motor_cam->setPosition(INFINITY);                       // Set position to continuous motor (motor_cam)
  motor_cam->setVelocity(0);                              // Set velocity 0 rad/s (motor_cam)

  double angle = 0.0;
  double  base = 0.6;
  double  step = 0.002;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    const double *rPos = robotPos->getSFVec3f();

    for (int i = 0; i < 3; i++) current_pos[i] = rPos[i];
    camPos->setSFVec3f(current_pos);                      // Set new position to follower camera
    
    angle += step;
    if (angle == 2 * M_PI) angle = 0;
    
    motor_cam->setVelocity(base * sin(angle));            // Set new velocity to motor camera
  };

  delete robot;
  return 0;
}
