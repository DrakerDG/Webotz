#ifndef UI_HPP
#define UI_HPP

#include <webots/Supervisor.hpp>      // Header of Webots Supervisor
#include <webots/Camera.hpp>          // Header of Webots Camera
#include <webots/Display.hpp>         // Header of Webots Display
#include "State.hpp"                  // Header of State
#include <string>

class Display_cam {
public:
  Display_cam(webots::Supervisor *robot, int timeStep);     // Module to initialize display and camera
  void drawCursor(webots::Field *robotField) const;         // Draw the robot cursor
  void clearDisplay() const;                                // Clear the display
  void setColorDisplay(int color) const;                    // Set color in display
  void saveTrack(ModeState mode) const;                     // Save image of track finished
  std::string hms(double sec) const;                        // Format the time as hh:mm:ss
  void printStatus(webots::Supervisor *robot,               // Print the track controller status to the 3D viewport
                                double time0,
                               bool finished,
                              ModeState mode);
private:
  webots::Camera   *camera;                                 // Webots camera
  webots::Display *display;                                 // Webots display
};



#endif
