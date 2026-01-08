#ifndef STATE_HPP
#define STATE_HPP

enum class RobotState {     // Robot status class
  CALIBRATION,
  INITIAL_RUN,
  OPTIMIZED_RUN,
  STOP_RUN
};

enum class StopReason {     // Stop status class
  NONE,
  CALIBRATION_DONE,
  INITIAL_DONE,
  OPTIMIZED_DONE,
  PAUSE
};

#endif

