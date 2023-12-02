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
#define PIN_MOTOR_2B    P9_4  // 
#define PIN_SERVO_PWM   P9_2  // P5_2 //

// Drive Motor PWM Frequency
#define DRV_PWM_FREQ    20
#define MAX_RPM         200

// Wheel Constants
#define RADIUS          53.3    // [mm]
#define CIRCUMFERENCE   336.15  // [mm]
#define WHEEL_WIDTH     200     // distance between both wheels (width of robot)
#define MAX_SPEED       1.15    // [m/s]
#define RPMtoDC         0.005   // [1/200rpm]
#define MPStoDC         89.6    // [(1000mm/m * 60sec/min * 100%) / (53.3mm * 2 * pi * 200rpm)]

// Turning and Motion Defs
#define LEFT       -1
#define RIGHT       1

#define FORWARD     1
#define REVERSE    -1
#define BRAKE       0

#define CLOCKWISE  -1
#define COUNTERCW   1


extern struct MOTOR{
    char name;
    cyhal_pwm_t * motor_pwm[2];
    int duty;
    int direction;
    bool sig1active;
    bool sig2active;
} motorA, motorB;

/* Public Function API */
void motor_init(void);  // Needs specification for non-staff-demo projects

// Init drive motor pins for pwm signals.
void drive_motor_init(void);

// set specific input signal (1A, 2A, 1B, 2B) to run at a certain duty cycle.
void set_drive_motor_signal(struct MOTOR * motor, int signal, int duty);

// Kills all pwms going to drive motors
void kill_motor_signal();

// sets spin direction of single motor
void set_motor_direction(struct MOTOR *motor, int direction);

// set duty cycle of single motor
void set_motor_duty(struct MOTOR *motor, int duty);

// set travel direction, forward/reverse/brake
void set_drive_direction(int direction);

// set drive duty cycle
void set_drive_duty(int duty);

// updates all drive motor pwms with the corresponding values from the motor structs.
void drive_update();


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