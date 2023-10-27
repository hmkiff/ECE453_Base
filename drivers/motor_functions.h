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

// Pin definitions for the ECE453 Staff Dev board
#define PIN_MOTOR_DIR   P5_0
#define PIN_MOTOR_STEP  P5_1
#define PIN_MOTOR_PWM  P5_2

// Exported Global Variables
extern cyhal_pwm_t step_pwm_obj;
extern cyhal_pwm_t servo_pwm_obj;

/* Public Function API */
void motor_init(void);

void motor_set_pwm(int angle);
void motor_step_speed(int speed);
void motor_set_dir(bool direction);
void motor_free(void);

#endif