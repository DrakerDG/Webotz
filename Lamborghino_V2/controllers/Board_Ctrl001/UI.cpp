#include "UI.hpp"
#include <iomanip>                                             // Format library to use setw to set some formats with decimals
#include "Config.hpp"                                          // Config header with configuration constants (Scale and Track)

using namespace webots;

static std::string modeToStr(ModeState s) {                    // Function to convert the robot state to string
  switch (s) {
    case ModeState::FIRST:  return "1";
    case ModeState::SECOND: return "2";
    default:                return "0";
  }
}

Display_cam::Display_cam(Supervisor *robot, int timeStep) {    // Module to initialize display and camera

  // Initialize camera
  camera = robot->getCamera("camera");                         // Get device (camera)
  camera->enable(timeStep * 4);                                // Set time step x 4 to camera

  // Initialize displays
  display = robot->getDisplay("display");                      // Get device (display)
  display->attachCamera(camera);                               // Attach display to camera
  setColorDisplay(0xFFFF00);                                   // Set yellow color to display
}

void Display_cam::drawCursor(Field *robotField) const {        // Draw the robot cursor
  const double *robot0 = robotField->getSFVec3f();
  double x = display->getWidth()/2  + robot0[0] * SCALE;
  double y = display->getHeight()/2 - robot0[1] * SCALE;
  display->fillOval(x, y, 1, 1);
}

void Display_cam::clearDisplay() const {                       // Clear the display
  display->setAlpha(0.0);
  display->fillRectangle(0, 0, display->getWidth(), display->getHeight());
  display->setAlpha(1.0);
}

void Display_cam::setColorDisplay(int color) const {           // Set color in display
  display->setColor(color);
}

void Display_cam::saveTrack(ModeState mode) const {            // Save image of track finished
  display->imageCopy(0, 0, display->getWidth(), display->getHeight());
  std::string name = "track_00" + modeToStr(mode) + ".png";
  display->imageSave(0, name);
}

std::string Display_cam::hms(double sec) const {               // Format the time as hh:mm:ss
  int h = static_cast<int>(sec / 3600);
  int m = static_cast<int>(sec) % 3600 / 60;
  int s = static_cast<int>(sec) % 60;
  int c = static_cast<int>((sec - static_cast<int>(sec)) * 1000);

  std::ostringstream tm;
  tm << std::setfill('0')
     << std::setw(2) << h << ":"
     << std::setw(2) << m << ":"
     << std::setw(2) << s << "."
     << std::setw(3) << c;

  return tm.str();
}

void Display_cam::printStatus(Supervisor *robot,               // Print the track controller status to the 3D viewport
                                   double time0,
                                  bool finished,
                                 ModeState mode) {

  double tmr = robot->getTime() - time0;
  
  std::string ave_speed = "";
  
  if (finished) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.3f", (tmr > 1e-6) ? (TRACK / tmr) : 0.0);
    ave_speed = std::string("    Average Speed:   ") + buffer + " m/s";
  }

  std::string text = "Lap Time " + modeToStr(mode) + ":   " + hms(tmr) + ave_speed;

  switch (mode) {
  case ModeState::FIRST:
    robot->setLabel(0, text, 0, 0.85, 0.06, 0x00FF00, 0, "Lucida Console");           // Print the Lap time 1 and average speed of robot
  break;
  case ModeState::SECOND:
    robot->setLabel(1, text, 0, 0.89, 0.06, 0x00FF00, 0, "Lucida Console");           // Print the Lap time 2 and average speed of robot
  break;  
  }
}
