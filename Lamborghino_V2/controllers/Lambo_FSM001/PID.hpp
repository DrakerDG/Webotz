#ifndef PID_HPP
#define PID_HPP

class PID {
public:
  PID(double kp, double ki, double kd);            // Module to set PID keys (Kp, Ki and Kd)

  double update(double error, int timestep);       // Function to calculate and update the PID value
  void reset();                                    // Function to reset the PID value

private:
  double Kp, Ki, Kd;
  double prevError;
  double integral;
  double derivative;
};

#endif
