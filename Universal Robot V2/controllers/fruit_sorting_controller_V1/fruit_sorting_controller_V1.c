/*
 * File:          fruit_sorting_controller_V1.c
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <webots/camera_recognition_object.h>
#include <webots/motor.h>
#include <webots/supervisor.h>

#include <stdio.h>

#define TIME_STEP 32

enum State {WAITING, PICKING, ROTATING, DROPPING, ROTATE_BACK};

// This is the main program.
int main(int argc, char **argv) {
  wb_robot_init();
  
  int counter = 0, i = 0;
  int state = WAITING;
  const double target_positions[] = {-1.570796, -1.87972, -2.139774, -2.363176, -1.50971};
  
  double speed = 2.0;
  int model = 0;
  char fruit[20];
  int apple = 0;
  int orange = 0;
  
  char strP[100];

  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  //getting and declaring the 3 finger motors of the gripper 
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");

  
  //getting and declaring the robot motor
  WbDeviceTag ur_motors[5];
  ur_motors[0] = wb_robot_get_device("shoulder_pan_joint");
  ur_motors[1] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[2] = wb_robot_get_device("elbow_joint");
  ur_motors[3] = wb_robot_get_device("wrist_1_joint");
  ur_motors[4] = wb_robot_get_device("wrist_2_joint");

  //declaring and enabling the camera for recognitions
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2 * TIME_STEP);
  wb_camera_recognition_enable(camera, 2 * TIME_STEP);
  
  for (i = 0; i < 5; ++i)
    wb_motor_set_velocity(ur_motors[i], speed);

  WbDeviceTag distance_sensor = wb_robot_get_device("distance sensor");
  wb_distance_sensor_enable(distance_sensor, TIME_STEP);

  WbDeviceTag position_sensor = wb_robot_get_device("wrist_1_joint_sensor");
  wb_position_sensor_enable(position_sensor, TIME_STEP);
 

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    
    //get the camera object recognition details
    int number_of_objects = wb_camera_recognition_get_number_of_objects(camera);
    
    //Get and display the objects information
    const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

    if (number_of_objects > 0) {
      sprintf(fruit, "%s", objects[0].model);
      if (fruit[0] == 97) model = 1;
      else model = 0;
    }

    //switch cases for the different state of the arm
    if (counter <= 0) {
      switch (state) {
        case WAITING:
          if (wb_distance_sensor_get_value(distance_sensor) < 500) {
            state = PICKING;
            if (model == 1) apple +=1;
            else orange +=1; 
            counter = 8;
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], 0.52);
          }
          break;
        case PICKING:
          for (i = model; i < 5; ++i)
            wb_motor_set_position(ur_motors[i], target_positions[i]);
          state = ROTATING;
          break;
        case ROTATING:
          if (wb_position_sensor_get_value(position_sensor) < -2.3) {
            counter = 8;
            state = DROPPING;
            for (i = 0; i < 3; ++i)
              wb_motor_set_position(hand_motors[i], wb_motor_get_min_position(hand_motors[i]));
          }
          break;
        case DROPPING:
          for (int i = model; i < 5; ++i)
            wb_motor_set_position(ur_motors[i], 0.0);
          state = ROTATE_BACK;
          break;
        case ROTATE_BACK:
          if (wb_position_sensor_get_value(position_sensor) > -0.1) {
            state = WAITING;
          }
          break;
      }
    }
    counter--;

    sprintf(strP, "Oranges: %d", orange);
    wb_supervisor_set_label(0, strP, 0.45, 0.96, 0.06, 0x5555ff, 0, "Lucida Console");
    
    sprintf(strP, "Apples : %d", apple);
    wb_supervisor_set_label(1, strP, 0.3, 0.96, 0.06, 0x5555ff, 0, "Lucida Console");
    
  }
  wb_robot_cleanup();

  return 0;
}
