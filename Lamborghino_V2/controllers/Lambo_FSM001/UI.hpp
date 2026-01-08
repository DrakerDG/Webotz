#ifndef UI_HPP
#define UI_HPP

#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Node.hpp>            // Header of Webots Node
#include <webots/Camera.hpp>          // Header of Webots Camera
#include "Sensors.hpp"                // Header of Sensors
#include "State.hpp"                  // Header of State

class Display_cam {
public:
  Display_cam(webots::Supervisor *robot, int timeStep);     // Module to initialize display, camera and self node
  void printStatus(webots::Supervisor *robot,               // Module to print robot name, speed, curve marks, robot state, pauses, and sensors values
                            const Sensors &s,
                               int leftMarks,
                              int rightMarks,
                            RobotState state,
                      StopReason stopReason);

private:
  webots::Camera *camera;                                   // Webots camera
  webots::Node     *self;                                   // Webots self node (robot)
};

#endif
