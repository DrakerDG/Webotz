#include "Sensors.hpp"

using namespace webots;

Sensors::Sensors(Robot *robot, int timeStep) {    // Module to initialize sensors

  // Initialize sensors
  start = robot->getDistanceSensor("start");     // Get device (sensor of start)
  start->enable(timeStep * 2);                   // Set time step x 2 to sensor of start

  goal = robot->getDistanceSensor("goal");       // Get device (sensor of goal)
  goal->enable(timeStep * 2);                    // Set time step x 2 to sensor of goal
}

int Sensors::start_getValue() const {           // Function to get the start sensor value
  return (int)start->getValue();
}

int Sensors::goal_getValue() const {            // Function to get the goal sensor value
  return (int)goal->getValue();
}

