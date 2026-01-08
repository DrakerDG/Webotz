#ifndef SENSORS_HPP
#define SENSORS_HPP

#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>

class Sensors {
public:
  Sensors(webots::Robot *robot, int timeStep);      // Module to initialize sensors
  void updateCalibration();                         // Module to calibration sensors
  bool calibrationDone() const;                     // Function of the calibration status
  int normalize(int i) const;                       // Function to normalize the sensor value, using min, max and normal target (NR_GS)
  double lineError();                               // Function to get the line error
  int getColor(int i) const;                        // Function to get the sensor color
  int getValue(int i) const;                        // Function to get the sensor value
  int mrkValue(int i);                              // Function to get the curve mark value
  void mrkReset();                                  // Function to reset the curve mark
  void mrkUpdate();                                 // Function to update the digital footprint of curve mark
  bool mrkLeft() const;                             // Function to confirm the Left curve marking using the digital footprint
  bool mrkRight() const;                            // Function to confirm the Right curve marking using the digital footprint
//  int chkValue(int i);                              // Function to get the curve mark value
//  void chkReset();                                  // Function to reset the curve mark
//  void chkUpdate();                                 // Function to update the digital footprint of curve mark
//  bool chkLeft() const;                             // Function to confirm the Left curve marking using the digital footprint
//  bool chkRight() const;                            // Function to confirm the Right curve marking using the digital footprint
  void cout_max_min();                              // Function to show the max and min sensor value    <--- to debug
  void cout_new();                                  // Function to show the normalized sensor value     <--- to debug
private:
  webots::DistanceSensor *gs[8];                    // Webots distance sensor array
  int  gs_RGB[8];
};

#endif
