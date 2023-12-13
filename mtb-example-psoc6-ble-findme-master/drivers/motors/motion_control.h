/*
 * motion_control.h
 *
 *  Created on: Dec 4, 2023
 *      Author: Harry Kiffel
 */

#ifndef MOTOR_CONTROL_H_
#define MOTOR_CONTROL_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "motor_functions.h"

extern struct ROBOT{
        // Position 
        struct VEC2D r_center_world;    //  Position: initialize at 0,0
        double theta;                         //  Robot angle: initialize at 0
        //  Velocity
        struct VEC2D v_center_world;    //  Velocity (world frame): initialize at 0,0
        double omega;                   //  Robot angular velocity: initialize at 0
        //  States for Wheel Speeds
        double left_wheel_speed;             
        double right_wheel_speed;        
        //  Keep track of how far the wheels have gone (meters). 
        double left_wheel_distance_traveled;  
        double right_wheel_distance_traveled;
        //  And how far they have turned (radians)
        double left_wheel_angle;
        double right_wheel_angle;
} bug;

struct PATHSPEC{
    double x0;
    double y0;
    double theta0;
    double Radius;
    double Length;
};

struct POSE{
    double x;
    double y;
    double theta;
};

struct WHL_CMD {
    float duration;
    float lspeed;
    float rspeed;
};

void rotateBot(float speed_mps, float deg_angle);

// Create a rotation matrix to translate the robot frame to the world-frame
struct MAT2 get_rotmat_body_to_world();

// update the wheel speeds of the robot struct
void set_wheel_speeds(double v_left, double v_right);

// update wheel speeds in robot struct based on robot velocities
void set_wheel_speeds_from_robot_velocities(double forward_velocity, double angular_velocity);

// adjusts an angle to fall within pi to negative pi
float fix_angle_pi_to_neg_pi(float angle);

// specifies the wheel speeds and duration for a line
struct WHL_CMD plan_line(float speed, float length);

// specifies the wheel speeds and duration for an arc
struct WHL_CMD plan_arc(float radius, float speed, float angle);

// specifies the wheel speeds and duration for a pivot
struct WHL_CMD plan_pivot(float omega, float angle);

// specifies the wheel speeds and duration for a pause
struct WHL_CMD plan_pause(float pause_time);

// specifies the line path to follow from A to B
struct PATHSPEC specify_line(float x0, float y0, float xf, float yf);

// specifies the arc path to follow from A to B
struct PATHSPEC specify_arc(float x0, float y0, float xf, float yf, float R, bool way);

void drive_line(float distance_m, float speed_mps);

// Tells the robot to drive in an arc with a 'turn_radius' at 'speed_mps' in 'direction'.
void drive_arc(float speed_mps, int direction);

void drive_angle(float speed_mps, int angle);

#endif