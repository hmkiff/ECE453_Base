/*
 * ece453.h
 *
 *  Created on: October 9, 2023
 *      Author: Harry Kiffel
 */

#ifndef MOTOR_H_
#define MOTOR_H_

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


/* Public Function API */
void motor_init(void);

void motor_set_pwm(int angle);
void motor_step_speed(int speed);
void motor_set_dir(bool direction);

#endif