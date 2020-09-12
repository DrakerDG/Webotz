/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/***************************************************************************
  e-puck_line -- Base code for a practical assignment on behavior-based
  robotics. When completed, the behavior-based controller should allow
  the e-puck robot to follow the black line, avoid obstacles and
  recover its path afterwards.
  Copyright (C) 2006 Laboratory of Intelligent Systems (LIS), EPFL
  Authors: Jean-Christophe Zufferey
  Email: jean-christophe.zufferey@epfl.ch
  Web: http://lis.epfl.ch
  This program is free software; any publications presenting results
  obtained with this program must mention it and its origin. You
  can redistribute it and/or modify it under the terms of the GNU
  General Public License as published by the Free Software
  Foundation; either version 2 of the License, or (at your option)
  any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307,
  USA.
***************************************************************************/

#include <stdio.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>

#include <sys/time.h> //gettimeofday()


// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 32  // [ms]

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7

WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
// *** TO BE ADAPTED TO YOUR ROBOT ***
const int PS_OFFSET_REALITY[NB_DIST_SENS] = {480, 170, 320, 500, 600, 680, 210, 640};

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
WbDeviceTag gs[NB_GROUND_SENS]; /* ground sensors */
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0};

// Motors
WbDeviceTag left_motor, right_motor;

// obstacle avoidance strategy
#define PS_A 280 //300
#define PS_B 100 //100
#define PS_C 80 //80
#define MAX_GS 840 //840
#define MIN_GS 300
#define NEW_GS 1000
#define OL_GS 10 // 100: Overlap of sensor limits
#define GOAL 260 //Green of goal

bool ontrack = TRUE;
bool avoiding = FALSE;
bool around = FALSE;
bool recovery = FALSE;
bool turnL = FALSE;
bool turnR = FALSE;
bool stopRobot = FALSE;
bool online = FALSE;

int lineL, lineR;

short gs_new[NB_GROUND_SENS] = {0, 0, 0};
// Test array
unsigned short maxGS[NB_GROUND_SENS]= {500, 500, 500};
unsigned short minGS[NB_GROUND_SENS]= {500, 500, 500};


////////////////////////////////////////////
//
//   New Reading Ground Sensors Module
//
////////////////////////////////////////////

unsigned long Position = 0;
long ErrorPosition = 0;

void ReadGroudSensors(void){
  online = false;
  unsigned long avgS = 0;
  unsigned int sumS = 0;
    
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);

//    Max & Min detection
//    if(gs_value[i]<minGS[i]) minGS[i]=gs_value[i];
//    if(gs_value[i]>maxGS[i]) maxGS[i]=gs_value[i];
    
    // linear Interpolation
    gs_new[i] = ((float)gs_value[i]-MIN_GS)/(MAX_GS-MIN_GS)*-NEW_GS+NEW_GS;

    // Limited values between 0 and 1000 (NEW_GS)
    if(gs_new[i]>NEW_GS) gs_new[i]=NEW_GS;
    if(gs_new[i]<0) gs_new[i]=0;
    
    if(gs_new[i]>200)online = TRUE;
    if(gs_new[i]>50){
      // Average groud sensor value
      avgS += (unsigned long)gs_new[i]*(i*NEW_GS);
      // Sum ground sensor value
      sumS += gs_new[i];
    }
  }
  if(online)Position = avgS/sumS; // Position Calculation
  else if(Position < NEW_GS)Position = 0; // Left Sensor Memory Position
  else Position = NEW_GS*2; // Right Sensor Memory Position
  
  // Error Position Calculation
  ErrorPosition = Position - NEW_GS;
  
//  printf("%4d   %4d   %4d   %4d   %5d   OnLine: %d \n", gs_new[0], gs_new[1], gs_new[2], (int)Position, (int)ErrorPosition, online);
//  printf("GS: %4d %4d %4d;   Max: %4d %4d %4d;   Min: %4d %4d %4d  \n", gs_value[0], gs_value[1], gs_value[2], maxGS[0], maxGS[1], maxGS[2], minGS[0], minGS[1], minGS[2]);
}


//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// LFM - Line Following Module
//
// This module implements a very simple, Braitenberg-like behavior in order
// to follow a black line on the ground. Output speeds are stored in
// lfm_speed[LEFT] and lfm_speed[RIGHT].

int lfm_speed[2];

#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.4 //0.4

void LineFollowingModule(void) {
  int DeltaS = 0;

//  DeltaS = ErrorPosition;
  DeltaS = gs_value[2] - gs_value[0];

  lfm_speed[LEFT] = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
  lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;

}

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------

////////////////////////////////////////////
// Main
int main() {
  int i, speed[2], ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

  /* intialize Webots */
  wb_robot_init();

  /* initialization */
  char name[20];
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name); /* proximity sensors */
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

// Mew Main loop 

  for (;;) {
    // Run one simulation step
    wb_robot_step(TIME_STEP);

    // read sensors value
    for (i = 0; i < NB_DIST_SENS; i++)
      ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
    ReadGroudSensors();
    // Speed initialization

    speed[LEFT] = 0;
    speed[RIGHT] = 0;

    // *** START OF SUBSUMPTION ARCHITECTURE ***

    if(ontrack){
      // LFM - Line Following Module
      LineFollowingModule();
      speed[LEFT] = lfm_speed[LEFT];
      speed[RIGHT] = lfm_speed[RIGHT];
      // Routines used when detecting the line
      if(online){
        // If the center sensor detects the line
        if(ErrorPosition>-5 && ErrorPosition<5){
          // If the number of times the left sensor detects the line is between 3 and 8,
          // it activates force crossing and deactivates the line detection routine
          if(lineL>2 && lineL<9){
            turnL = TRUE;
            ontrack = FALSE;
          }
          else turnL = FALSE;
          if(lineR>2 && lineR<9) turnR = TRUE;
          else turnR = FALSE;
          lineL = 0;
          lineR = 0;
        }
//      Counts how many times the line is detected with the left sensor        
        else if(ErrorPosition<-OL_GS)lineL++;
//      Counts how many times the line is detected with the right sensor  
        else if(ErrorPosition>OL_GS)lineR++;
      }
//    Routine braking and recovery when losing the line
      else{
        if(ErrorPosition == -NEW_GS){
          speed[LEFT] = -200;
          speed[RIGHT] = 200;
        }
        if(ErrorPosition == NEW_GS){
          speed[LEFT] = 200;
          speed[RIGHT] = -200;
        }
      }
    }
//  Crossover routine used for bifurcation
    else{
      if(turnL && !stopRobot){
        if(ErrorPosition < NEW_GS){
          speed[LEFT] = -200;
          speed[RIGHT] = 200;
        }
        else{
          turnL = FALSE;
          ontrack = TRUE;
        }
      }
    }

//  When detecting the green color with the three gs sensors, the robot stops (Goal!!!)
    if(gs_new[1]>(GOAL-10) && gs_new[1]<(GOAL+10)){
      if(gs_new[0]>(GOAL-10) && gs_new[0]<(GOAL+10)){
        if(gs_new[2]>(GOAL-10) && gs_new[2]<(GOAL+10)){
          ontrack = FALSE;
          stopRobot = TRUE;
        }
      }
    }

//    printf("%4d   %4d   %4d   %5d   OnLine: %d   LineL: %2d   LineR: %2d   Left: %d   Right: %d  \n", gs_new[0], gs_new[1], gs_new[2], (int)ErrorPosition, online, lineL, lineR, turnL, turnR);

//  When it is detecting an obstacle. PSA is a detected light intensity
    if(ps_value[7] > PS_A || ps_value[0] > PS_A){
      avoiding = TRUE;
      ontrack = FALSE;
    }

//  The robot rotates until the ps5 sensor detects the object
    if(avoiding){
      if(ps_value[5] < (PS_A-20)){
        speed[LEFT] = 200;
        speed[RIGHT] = -200;
      }
      else if(ps_value[5] >= PS_A){
        avoiding = FALSE;
        around = TRUE;
      }
    }

//  The robot circles the object until it detects the line again.    
    if(around){
      if(ps_value[5] < (PS_A-20) && ps_value[6] < PS_B){
        speed[LEFT] = -200;
        speed[RIGHT] = 100;
      }
      else{
        speed[LEFT] = 200;
        speed[RIGHT] = 200;
      }
      if(gs_value[0]<400 || gs_value[1]<400 || gs_value[2]<400){
        around = FALSE;
        recovery = TRUE;
      }
    }   

//  The robot rotates until the ps5 sensor stops detecting the obstacle     
    if(recovery){
      if(ps_value[5] > PS_C){
        speed[LEFT] = 200;
        speed[RIGHT] = -200;
      }
      else{
        recovery = FALSE;
        ontrack = TRUE;
      }
    }

//  Debug Console Print
    if(stopRobot) printf("Goal!!!  \n");
    else printf("Ontrack: %d   Avoiding: %d   Around: %d   Recovey: %d   \n", !(avoiding || around || recovery), avoiding, around, recovery);

//    printf("GS[0] = %4d   GS[1] = %4d    GS[2] = %4d   \n", gs_value[0], gs_value[1], gs_value[2]);
    // Speed computation
    wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);

  }
  return 0;
}
