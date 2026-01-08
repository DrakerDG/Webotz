#include "PID.hpp"

PID::PID(double kp, double ki, double kd) {                         // Module to set PID keys (Kp, Ki and Kd)
  Kp = kp;
  Ki = ki;
  Kd = kd;
  prevError = 0;
  integral = 0;
  derivative = 0;
}

double PID::update(double error, int timestep) {                    // Function to calculate and update the PID value
  integral   = integral * (2 / 3) + error * timestep / 1000;
  derivative = derivative * 0.5 + (error - prevError) /timestep * 1000;
  prevError  = error;

  return Kp * error + Ki * integral + Kd * derivative;
}

void PID::reset() {                                                 // Function to reset the PID value
  prevError = 0;
  integral = 0;
  derivative = 0;
}

