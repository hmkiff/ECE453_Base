/*
 * ece453.h
 *
 *  Created on: October 9, 2023
 *      Author: Harry Kiffel
 */

#ifndef MOTOR_FUNCTIONS_H_
#define MOTOR_FUNCTIONS_H_

#include <stdlib.h>
#include <stdio.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

// Pin definitions for drive motors
#define PIN_MOTOR_1A    P10_6 // 
#define PIN_MOTOR_2A    P10_4 // 
#define PIN_MOTOR_1B    P10_0 // 
#define PIN_MOTOR_2B    P9_4 // 

// Drive Motor PWM Frequency
#define DRV_PWM_FREQ    20
#define MAX_RPM         200

// Wheel Constants
// radius: 53.3 mm
// max speed 200rpm
// 200rpm * 2pi / 60 = 20.94395 rad/s
// speed: rad/s * radius = 1158.200435 mm/s ~ 1.158 m/s of one wheel.
// 1m/s -> rpm -> duty cycle = (1000mm / 53.3mm) * 60 / (2pi) = 179.16128 rpm ~ 86%
#define CIRCUMFERENCE   336.15  // [mm]
#define WHEEL_WIDTH     200     // distance between both wheels (width of robot)
#define MAX_SPEED       1.15    // [m/s]
#define RPMtoDC         0.005   // [1/200rpm]
#define MPStoDC         89.6    // [(1000mm/m * 60sec/min * 100%) / (53.3mm * 2 * pi * 200rpm)]

// Turning and Motion Defs
#define LEFT    -1
#define RIGHT    1
#define FORWARD  1
#define REVERSE -1
#define BRAKE    0
#define STRAIGHT 0


// Pin definitions for the ECE453 Staff Dev board
#define PIN_MOTOR_DIR   P5_0
#define PIN_MOTOR_STEP  P5_1
#define PIN_MOTOR_PWM   P0_4

extern struct MOTOR{
    char name;
    cyhal_pwm_t * motor_pwm[2];
    int duty;
    int direction;
} motorA, motorB;

// Exported Global Variables
extern cyhal_pwm_t step_pwm_obj;
extern cyhal_pwm_t servo_pwm_obj;
extern cyhal_pwm_t drive1A_pwm_obj;
extern cyhal_pwm_t drive2A_pwm_obj;
extern cyhal_pwm_t drive1B_pwm_obj;
extern cyhal_pwm_t drive2B_pwm_obj;

//extern struct MOTOR motorA;
//extern struct MOTOR motorB;

/* Public Function API */
void motor_init(void);  // Needs specification for non-staff-demo projects

// Init drive motor pins for pwm signals.
void drive_motor_init(void);

// set specific input signal (1A, 2A, 1B, 2B) to run at a certain duty cycle.
void set_drive_motor_signal(struct MOTOR * motor, int signal, int duty);

// sets direction of single motor
void set_drive_motor_direction(struct MOTOR * motor, int direction);

// sets percent throttle of single motor
void set_drive_motor_speed(struct MOTOR * motor, int duty);

// sets speed of single motor given rpm
void set_drive_motor_speed_rpm(struct MOTOR * motor, int speed_rpm);

// sets drive move direction for both motors
void set_drive_move_direction(int move_dir);

// sets turn direction.
void set_drive_turn_direction(int turn_dir);

// sets percent throttle of both motors
void set_drive_speed(int duty);

// sets speed of both motors given rpm
void set_drive_speed_rpm(int speed_rpm);

// Tells the robot to drive in a straight line of 'distance_cm' [cm] at a 'speed_mps' [m/s]
void drive_line(int distance_cm, float speed_mps);

// Tells the robot to drive in an arc with a 'turn_radius' at 'speed_mps' in 'direction'.
void drive_arc(float turn_radius, float speed_mps, int direction);

// prints contents of MOTOR struct.
void print_motor(struct MOTOR * motor);






// generic motor/servo functions
void motor_set_pwm(int angle);
void motor_step_speed(int speed);
void motor_set_dir(bool direction);


//void motor_free(void);

#endif