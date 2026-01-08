// File:          Lambo_FSM001.cpp  |  Main  |
// Date:          January 2026
// Description:   Controller in C++ to 3D model of Lamborghino robot (Based on https://lamborghino.com/)
// Author:        DrakerDG
// Version:       2.2 Beta
//        ____             __             ____  ______
//       / __ \_________ _/ /_____  _____/ __ \/ ____/
//      / / / / ___/ __ `/ //_/ _ \/ ___/ / / / / __  
//     / /_/ / /  / /_/ / ,< /  __/ /  / /_/ / /_/ /  
//    /_____/_/   \__,_/_/|_|\___/_/  /_____/\____/   
//                                                                                                             

#include <iostream>                   // Input/Output Streams to use cout to debug routines
#include <iomanip>                    // Format library to use setw to set some formats with decimals
#include <algorithm>                  // Algorithm library
#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Motor.hpp>           // Header of Webots Motor
#include <webots/Camera.hpp>          // Header of Webots Camera

#include "Config.hpp"                 // Config header with configuration constants (thresholds, speeds)
#include "State.hpp"                  // State header with robot state and stop state (pause)
#include "Sensors.hpp"                // Sensors header with class to calibrate, normalize and determine curve marking detection
#include "PID.hpp"                    // PID header with class to set PID keys and calculate PID error
#include "SpeedCtrl.hpp"              // SpeedCtrol header with class to set the limit and target speed to update the base speed robot 
#include "Sound.hpp"                  // Sound header with class to play sounds
#include "UI.hpp"                     // User Interface header with class to print status of robot in 3D viewport

using namespace webots;

// Main module of Webots
int main() {
  Supervisor *robot = new Supervisor();          // To supervisor mode
  int timeStep = robot->getBasicTimeStep();      // Set time step

  // Initialize motors
  SpeedCtrl speedCtrl(robot, 0);                 // Get device (motors) through SpeedCtrl class
  speedCtrl.setTarget(0);                        // Set target speed in 0 rad/s

  // Initialize sensors
  Sensors sensors(robot, timeStep);              // Get device (sensors) through Sensor class
  
  // Initialize camera
  Display_cam display_cam(robot, timeStep);      // Get device (camera) through Display_cam class
  
  // Initialize speaker
  Sound sound(robot);                            // Get device (speaker) through Sound class

  RobotState nextState = RobotState::STOP_RUN;   // Set next state to stop run
  StopReason stopReason = StopReason::NONE;      // Set stop reason to none
  RobotState state = RobotState::CALIBRATION;    // Set current state to calibration

  //PID pid(26.0, 2.6, 0.0026);                  // Unstable PID
  PID pid(22.0, 1.0, 0.002);                     // Stable PID

  int rightMarks  = 0;                           // Reset right marks counter
  int leftMarks   = 0;                           // Reset left marks counter
  int stopCounter = 0;                           // Reset stop marks counter
 
  struct Segment {                               // Declare the segment structure (between marks)
    bool isStraight;                             // It is true if the line is straight
    int  lengthSamples;                          // Amount of samples (time steps) in the segment
    int  curve_index;
  };
  std::vector<Segment> trackMap;                 // Define the track map vector
  
  double segmentCurvatureAcc = 0.0;              // To add every curvature value (diff)
  int avgCurv = 0;                               // To calculate the average curvature
  double diff = 0.0;                             // To calculate the difference between leftSpeed and rightSpeed
  int segmentSamples = 0;                        // Quantity of samples per segment
  int segmentsBase = 0;                          // Quantity of samples per base segment
  int remaining = 0;                             // Remaining of base segments
  bool current_segment = true;                   // current segment (true = is straight)
  bool next_segment   = true;                    // Next segment (true = is straight)

  while (robot->step(timeStep) != -1) {                     // Main loop in Webots

    double error = sensors.lineError();                     // Calculate the error sensed using sensors class
    double correction = pid.update(error, timeStep);        // Calculate the correction error using PID class
    
    speedCtrl.update();                                     // Update the speed limit gradually
  
    int baseSpeed = speedCtrl.getSpeed();                   // Get the current speed
    int limit = speedCtrl.getLimit();                       // Get the limit speed
    
    int leftSpeed  = baseSpeed - correction;                // Determine the  left speed with correction
    int rightSpeed = baseSpeed + correction;                // Determine the right speed with correction   
  
    leftSpeed  = std::clamp(leftSpeed,  -limit, limit);     // Limits the  left speed within a defined range
    rightSpeed = std::clamp(rightSpeed, -limit, limit);     // Limits the right speed within a defined range

    switch (state) {
    
    case RobotState::CALIBRATION:                                       // Calibration state to determine min and max sensors value
      sensors.updateCalibration();                                      // Update the calibration of each sensor, recording the maximum and minimum values ​​detected
      
      speedCtrl.setSpeed(SP000, -SP000);                                // Set the calibration speed (motor_L and motor_R)

      if (sensors.mrkRight()) sound.play(0);                            // If the right sensor detects the line, it plays a sound
    
      if (sensors.calibrationDone()) {                                  // If the calibration has finished
        pid.reset();                                                    // Reset PID
        sensors.mrkReset();                                             // Reset curve mark counter
        speedCtrl.setTarget(0);                                         // Set the target speed to 0 rad/s
        stopReason = StopReason::CALIBRATION_DONE;                      // Set stop reason to calibrate done
        nextState  = RobotState::INITIAL_RUN;                           // Set next state to initial run
        state      = RobotState::STOP_RUN;                              // Set current state to stop run
        //sensors.cout_max_min();                                       //   <--- to debug
      }
      
      break;

    case RobotState::INITIAL_RUN:                                       // Initial run state, when the robot maps the entire track
                                                                        // records all the curve marks and each segment between the marks and their number of samples
      speedCtrl.setSpeed(leftSpeed, rightSpeed);                        // Set the calibration speed (motor_L and motor_R)
      
      diff = std::abs(leftSpeed - rightSpeed);                          // Calculate the speed difference (curvature) between the left and right motors
      segmentCurvatureAcc += diff;                                      // Accumulate the speed differences of each segment
      segmentSamples++;                                                 // Count the number of time steps for each segment
      
      if (sensors.mrkLeft()) {                                          // If detect left mark
        leftMarks++;

        if (segmentSamples > 0) {
          avgCurv = std::round(segmentCurvatureAcc / segmentSamples);   // Calculate the average curvature of the segment
    
          // Record the segment log
          trackMap.push_back({
            avgCurv < C_THOLD,
            segmentSamples,
            avgCurv
          });
        }
        //std::cout << std::setw(5) << segmentSamples << std::endl;     //   <--- to debug
        //std::cout << std::setw(5) << avgCurv << std::endl;            //   <--- to debug
        
        // Restart for the next segment 
        segmentCurvatureAcc = 0.0;                                      // Reset the curvature value
        segmentSamples = 0;                                             // Reset the steps counter
        avgCurv = 0;                                                    // Resets the average curvature value
            
        sound.play(0);                                                  // Plays mark sound
      }
      if (sensors.mrkRight()) {                                         // If detect right mark
        rightMarks++;                                                   // Increase the number of right marks
        sound.play(0);                                                  // Plays mark sound
      }
      //sensors.cout_new();                                             //   <--- to debug
      if (rightMarks == 2) {                                            // If the number of right marks is 2
        speedCtrl.setTarget(0);                                         // Set the target speed to 0 rad/s
        stopReason = StopReason::INITIAL_DONE;                          // Update the pause (stop) reason
        nextState  = RobotState::OPTIMIZED_RUN;                         // Update the following status
        state      = RobotState::STOP_RUN;                              // Update the current status        
      }
      break;

    case RobotState::OPTIMIZED_RUN:                                     // Optimize run state
      speedCtrl.setSpeed(leftSpeed, rightSpeed);                        // Set the calibration speed (motor_L and motor_R)
      
      if (leftMarks > 0) {                                              // If I can already detect at least the first mark of curvature
        remaining = segmentsBase - segmentSamples;                      // Calculate the remaining segments
        segmentSamples++;                                               // Count the segments that occurred
        //std::cout << remaining << std::endl;                          // Segments remaining <--- to debug
        if (current_segment && !next_segment) {                         // If the next segment is a straight line but the one after it isn't
          if (remaining < BRAKE_M) {                                    // If the remaining amount is less than the braking threshold --> slow down (brake)
            speedCtrl.setTarget(SP001);                                 // Set the target speed to SP001 (low speed)
          }
        }
      }
      
      if (sensors.mrkLeft()) {                                          // If detect left mark
        leftMarks++;                                                    // Increase the number of left marks
        const Segment &seg = trackMap[leftMarks];                       // Gets the record of the current segment
        const Segment &next = trackMap[leftMarks + 1];                  // Gets the record of the next segment
        segmentsBase = seg.lengthSamples;                               // Copy the number of steps from the current segment
        current_segment = seg.isStraight;                               // Copies the line value (true / false) of the current segment
        next_segment   = next.isStraight;                               // Copies the line value (true / false) of the next segment
        if (current_segment) {                                          // If the current segment is a straight line (true)
          segmentsBase = int(FRAC * segmentsBase);                      // Calculate the fraction of the number of steps in the current segment
          speedCtrl.setTarget(SP003);                                   // Set the target speed to SP003 (high speed)
          //std::cout << "Fast" << std::endl;      //   <--- to debug
        }
        else {                                                          // If the current segment is not a straight line (false)
          speedCtrl.setTarget(SP001);                                   // Set the target speed to SP001 (low speed)
          //std::cout << "Slow" << std::endl;      //   <--- to debug
        }
        segmentSamples = 0;                                             // Reset the steps counter 
        sound.play(0);                                                  // Plays mark sound
      }
    
      if (sensors.mrkRight()) {                                         // If detect right mark
        rightMarks++;                                                   // Increase the number of right marks
        sound.play(0);                                                  // Plays mark sound
      }
      if (rightMarks == 2) {                                            // If the number of right marks is 2
        speedCtrl.setTarget(0);                                         // Set the target speed to 0 rad/s
        stopReason = StopReason::OPTIMIZED_DONE;                        // Update the pause (stop) reason
        nextState  = RobotState::STOP_RUN;                              // Update the following status  <-- the robot stops
        state      = RobotState::STOP_RUN;                              // Update the current status
      }
      break;

    case RobotState::STOP_RUN:                                          // Stop run state, when the robot pauses or stops
      speedCtrl.setSpeed(leftSpeed, rightSpeed);                        // Set the calibration speed (motor_L and motor_R)

      stopCounter++;                                                    // Increases the pause counter
    
      if (stopCounter >= STOP_DLY) {                                    // If the pause counter is longer than the stop time
        stopCounter = 0;                                                // Reset the pause counter
    
        if (stopReason != StopReason::OPTIMIZED_DONE) {                 // If the reason for the pause is not to complete the optimization run
          leftMarks  = 0;                                               // Reset the left marks
          rightMarks = 0;                                               // Reset the right marks
          segmentSamples = 0;                                           // Reset the steps counter
          speedCtrl.setTarget(SP001);                                   // Set the target speed to SP001 (low speed)
          state = nextState;                                            // Update the current status
        }
        // If it is OPTIMIZED_DONE → it remains stopped
      }
          
      
      break;
    }
    
    display_cam.printStatus(robot, sensors,                             // Print robot name, speed, curve marks, robot state, pauses, and sensors values
                leftMarks, rightMarks,
                state, stopReason);    
      
  }

  delete robot;
  return 0;
}
