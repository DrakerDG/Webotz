// File:          Board_Ctrl001.cpp |  Main  |
// Date:          January 2026
// Description:   Controller in C++ to board control (Start and Goal)
// Author:        DrakerDG
// Version:       2.2
//        ____             __             ____  ______
//       / __ \_________ _/ /_____  _____/ __ \/ ____/
//      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
//     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
//    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
//                                                                                                             

#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include "Config.hpp"                 // Config header with configuration constants (thresholds, speeds)
#include "State.hpp"                  // State header with robot state and stop state (pause)
#include "Sensors.hpp"                // Sensors header with class to calibrate, normalize and determine curve marking detection
#include "UI.hpp"                     // User Interface header with class to print status of robot in 3D viewport

using namespace webots;

int main(int argc, char **argv) {

  Supervisor *robot = new Supervisor();                    // create the Robot instance
 
  int timeStep = (int)robot->getBasicTimeStep();           // get the time step of the current world
  
  Node *robotNode = robot->getFromDef("Lamborghino");            // Get the node by DEF
  Field *robotField = robotNode->getField("translation");  // Get the "translation" field
  
  // Initialize sensors
  Sensors sensors(robot, timeStep);                        // Get device (start and goal sensors) through Sensor class

  // Initialize display and camera
  Display_cam display_cam(robot, timeStep);                // Get device (display and camera) through UI class

  CheckState state = CheckState::WAITING;
  ModeState   mode =  ModeState::FIRST;
  double time0 = robot->getTime();
  display_cam.printStatus(robot, time0, false, mode);
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    
    switch (state) {
    case CheckState::WAITING:
      if (sensors.start_getValue() > T_HOLD) {
        // std::cout << std::setw(5) << sensors.start_getValue() << std::endl;
        state = CheckState::RUNNING;
        time0 = robot->getTime();
        if( mode == ModeState::SECOND) {
          display_cam.clearDisplay();
          display_cam.setColorDisplay(0x00FF00);
        }
      }
    break;
    case CheckState::RUNNING:
      display_cam.drawCursor(robotField);
      display_cam.printStatus(robot, time0, false, mode);
      if (sensors.goal_getValue() > T_HOLD) {
        display_cam.printStatus(robot, time0, true, mode);
        // std::cout << std::setw(5) << sensors.goal_getValue() << std::endl;
        display_cam.saveTrack(mode);
        state = CheckState::WAITING;
        if( mode == ModeState::FIRST) {
            mode = ModeState::SECOND;
        }
      }
    break;
    }
  };

  delete robot;
  return 0;
}
