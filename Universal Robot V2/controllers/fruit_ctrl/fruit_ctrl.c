/*
 * File:          fruit_ctrl.c
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64
int main(int argc, char **argv) {
  int i = 0;
  int j = 8;
  int k = 8;
  int fr = 1;
  int max = 50;  
  char name[20];

  wb_robot_init();

  //declaring and enabling the camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);

  WbNodeRef fruit;
  WbFieldRef fruit_trans_field;
  double fruit_initial_translation[3] = {0.570002,2.85005,0.349962};

  while (wb_robot_step(TIME_STEP) != -1) {
    
    if (wb_robot_get_time() > 7.5){
      if (i == 0){
        if (fr > 0){
          fr = 1 + (rand() % 2);
          if (j == max) fr = 2;
          if (k == max) fr = 1; 
          if ((fr == 1) && (j < max)) {
            sprintf(name, "apple%d", j);
            j += 1;
          }
          if ((fr == 2) && (k < max)) {
            sprintf(name, "orange%d", k);
            k += 1;
          }
          fruit = wb_supervisor_node_get_from_def(name);
          fruit_trans_field = wb_supervisor_node_get_field(fruit, "translation");
          wb_supervisor_field_set_sf_vec3f(fruit_trans_field, fruit_initial_translation);
          if ((j == max) && (k == max)) fr = 0;
        }
      }
      i += 1;
      if (i == 120) i = 0;
    }
      
  };

  wb_robot_cleanup();

  return 0;
}
