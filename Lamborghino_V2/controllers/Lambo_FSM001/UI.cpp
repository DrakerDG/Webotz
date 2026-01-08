#include "UI.hpp"
#include <cmath>                                              // Math library to make math operations like sqrt 
#include <iomanip>                                            // Format library to use setw to set some formats with decimals

using namespace webots;

static std::string stateToStr(RobotState s) {                 // Function to convert the robot state to string
  switch (s) {
    case RobotState::CALIBRATION:   return "Calibration";
    case RobotState::INITIAL_RUN:   return "Initial Run";
    case RobotState::OPTIMIZED_RUN: return "Optimized Run";
    case RobotState::STOP_RUN:      return "Stop Run";
    default:                        return "Unknown";
  }
}

static std::string stopReasonToStr(StopReason r) {            // Function to convert the stop reason (pauses) to string
  switch (r) {
    case StopReason::NONE:              return "";
    case StopReason::CALIBRATION_DONE:  return "Calibration Done";
    case StopReason::INITIAL_DONE:      return "Initial Run Done";
    case StopReason::OPTIMIZED_DONE:    return "Finished";
    case StopReason::PAUSE:             return "Pause";
    default:                            return "";
  }
}

Display_cam::Display_cam(Supervisor *robot, int timeStep) {    // Module to initialize camera

  self = robot->getSelf();                                     // Get self robot
  
  // Initialize camera
  camera = robot->getCamera("camera");                         // Get device (camera)
  camera->enable(timeStep * 4);                                // Set time step x 4 to camera

}

void Display_cam::printStatus(Supervisor *robot,                                                 // Module to print robot name, speed, curve marks, robot state, pauses, and sensors values
                 const Sensors &s,
                 int leftMarks,
                 int rightMarks,
                 RobotState state,
                 StopReason stopReason) {

  const double *v = self->getVelocity();
  double speed = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

  char buffer[16];
  snprintf(buffer, sizeof(buffer), "%.2f", speed);
  std::string text = "Robot: " + robot->getName() + " | Speed: " + buffer + " m/s";
  
  robot->setLabel(0, text, 0, 0.93, 0.06, 0x00FF00, 0, "Lucida Console");           // Print name and speed of robot
  
  std::string stateText = "State: " + stateToStr(state);

  if (state == RobotState::STOP_RUN && stopReason != StopReason::NONE) {
    stateText += " (" + stopReasonToStr(stopReason) + ")";
  }

  std::ostringstream oss;
  oss << "Left marks: " << std::setw(2) << leftMarks
    << " | Right marks: " << std::setw(2) << rightMarks;
  std::string marks = oss.str() + " | " + stateText;

  robot->setLabel(1, marks, 0, 0.97, 0.06, 0x00FF00, 0, "Lucida Console");          // Print curve marks, robot state and pauses
  
  robot->setLabel(2, "█", 0.01, 0.01, 0.09, s.getColor(7), 0, "Lucida Console");    // Print status color of left sensor
  robot->setLabel(3, "L",  0.04, 0.01, 0.1,  0x00FF00, 0, "Lucida Console");

  for (int i = 0; i < 6; i++)
    robot->setLabel(i + 4, "█", 0.17 - i * 0.02, 0.01, 0.09,                        // Print status color of front sensors
                                        s.getColor(i + 1), 0, "Lucida Console");

  robot->setLabel(10,  "R",  0.20, 0.01, 0.1,  0x00FF00, 0, "Lucida Console");
  robot->setLabel(11, "█",  0.23, 0.01, 0.09, s.getColor(0), 0, "Lucida Console");  // Print status color of right sensor
}
