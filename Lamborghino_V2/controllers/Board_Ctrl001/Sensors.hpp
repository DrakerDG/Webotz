#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>

class Sensors {
public:
  Sensors(webots::Robot *robot, int timeStep);      // Module to initialize sensors
  int start_getValue() const;                       // Function to get the start sensor value
  int  goal_getValue() const;                       // Function to get the goal sensor value
  
private:
  webots::DistanceSensor *start;                    // Webots distance sensor of start
  webots::DistanceSensor  *goal;                    // Webots distance sensor of goal
};

#endif
