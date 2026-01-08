#include <iomanip>                                // Input/Output Streams to use cout to debug routines
#include "Sensors.hpp"
#include "Config.hpp"                             // Config header with configuration constants (thresholds, speeds)

using namespace webots;

int gs_min[8];                                    // Min value of every sensor (Calibration)
int gs_max[8];                                    // Max value of every sensor (Calibration)
int gs_new[8];                                    // New normalized value of every sensor
int calibSteps = 0;                               // Calibration steps counter
const int CALIB_STEPS = 570;                      // Time steps to calibrate  |  570 (4 times): SP000 = 20 rad/s  |
int weights[6] = { -3, -2, -1, 1, 2, 3 };         // Weights of every sensor to detect line of track
int mrk[8];                                       // Digital footprint of curve marking sensors

Sensors::Sensors(Robot *robot, int timeStep) {    // Module to initialize sensors
  for (int i = 0; i < N_SEN; i++) {
    std::string name = "QTR" + std::to_string(i);
    gs[i] = robot->getDistanceSensor(name);
    gs[i]->enable(timeStep);
    gs_RGB[i] = 0x000000;                         // Setting default color to show the status of every sensor
  }
}

void Sensors::updateCalibration() {               // Module to calibration sensors
  for (int i = 0; i < N_SEN; i++) {
    int v = gs[i]->getValue();
    if (calibSteps == 0) {
      gs_min[i] = v;
      gs_max[i] = v;
    } else {
      if (v < gs_min[i]) gs_min[i] = v;
      if (v > gs_max[i]) gs_max[i] = v;
    }
  }
  calibSteps++;
}

bool Sensors::calibrationDone() const {           // Function of the calibration status
  return calibSteps >= CALIB_STEPS;
}

int Sensors::normalize(int i) const {             // Function to normalize the sensor value, using min, max and normal target (NR_GS)
  if (gs_max[i] == gs_min[i]) return gs[i]->getValue();
  int v = gs[i]->getValue();
  if (v < gs_min[i]) v = gs_min[i];
  else if (v > gs_max[i]) v = gs_max[i];
  return NR_GS * (v - gs_min[i]) / (gs_max[i] - gs_min[i]);
}

double Sensors::lineError() {                     // Function to get the line error
  int sum = 0;
  int count = 0;
  mrkUpdate();
  mrk[0] = 0;                                     // Reset the checkmark
  for (int i = 0; i < N_SEN; i++) {
    gs_new[i] = normalize(i);
    if (gs_new[i] > THOLD) {
      if (i > 0 && i < (N_SEN - 1)) {
        sum += weights[i - 1];
        count++;
      }
      else {
        mrk[0] += mrkValue(i);
      }
      gs_RGB[i] = 0xFFFFFF;
    } 
    else {
      gs_RGB[i] = 0x000000;
    }
  }
  // std::cout << mrk[0] << mrk[1] << mrk[2] << mrk[3] << mrk[4] << mrk[5] << mrk[6] << mrk[7] << std::endl;  // <--to debug
  

  if (count == 0)
    return 0;                                     // Lost the line
  return (double)sum / count;
}

int Sensors::getColor(int i) const {              // Function to get the sensor color
  return gs_RGB[i];
}

int Sensors::getValue(int i) const {              // Function to get the sensor value
  return (int)gs[i]->getValue();
}

int Sensors::mrkValue(int i) {                    // Function to get the curve mark value
  if (i == 0) return 1;
  if (i == 7) return 2;
  return 0;
}

void Sensors::mrkReset() {                        // Function to reset the curve mark
  for (int i = 0; i < 8; i++) mrk[i] = 0;
}

void Sensors::mrkUpdate() {                       // Function to update the digital footprint of curve mark
  for (int i = 0; i < 7; i++) mrk[7 - i] = mrk[6 - i];
}

bool Sensors::mrkLeft() const {                   // Function to confirm the Left curve marking using the digital footprint
  return mrk[0] == 0 &&                           // Falling edge of left sensor value 
         mrk[1] == 2 &&
        (mrk[2] == 2 || mrk[2] == 0) && 
        (mrk[3] == 2 || mrk[3] == 0) &&
        (mrk[4] == 2 || mrk[4] == 0) &&
        (mrk[5] == 2 || mrk[5] == 0) &&
        (mrk[6] == 2 || mrk[6] == 0) &&
        (mrk[7] == 2 || mrk[7] == 0);
}

bool Sensors::mrkRight() const {                  // Function to confirm the Right curve marking using the digital footprint
  return mrk[0] == 0 &&                           // Falling edge of right sensor value
         mrk[1] == 1 &&
        (mrk[2] == 1 || mrk[2] == 0) &&
        (mrk[3] == 1 || mrk[3] == 0) &&
        (mrk[4] == 1 || mrk[4] == 0) &&
        (mrk[5] == 1 || mrk[5] == 0) &&
        (mrk[6] == 1 || mrk[6] == 0) &&
        (mrk[7] == 1 || mrk[7] == 0);
}

void Sensors::cout_max_min() {                    // Function to show the max and min sensor value <--- to debug
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_min[(N_SEN - 1) - i];
  }
  std::cout << std::endl;
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_max[(N_SEN - 1) - i];
  }
  std::cout << std::endl;  
}

void Sensors::cout_new() {                        // Function to show the normalized sensor value <--- to debug
  for (int i = 0; i < N_SEN; i++) {
    std::cout << std::setw(5) << gs_new[(N_SEN - 1) - i];
  }
  std::cout << std::endl;
}
