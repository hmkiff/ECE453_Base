/*
 * ece453.h
 *
 *  Created on: October 9, 2023
 *      Author: Harry Kiffel
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

// Pin definitions for the ECE453 Staff Dev board
#define PIN_MOTOR_DIR   P10_6
#define PIN_MOTOR_STEP  P5_3
#define PIN_MOTOR_STEP  P5_2

// Exported Global Variables


/* Public Function API */
void motor_handler(void* handler_arg, cyhal_gpio_event_t event);
void motor_init(void);


#endif