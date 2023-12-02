/*
 * ece453.h
 *
 *  Created on: October 20, 2023
 *      Author: Harry Kiffel
 */

#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#define PIN_TRIGGER P9_3
#define PIN_ECHO1   P10_1 // P9_1 // 
#define PIN_ECHO2   P10_5 // P9_0 // 

#define DELTA       5 // [cm]   // midpoint of us recvrs is 2.5cm from both
#define X_CORRECT   1 // [cm]   // midpoint of servo is 3.5 cm from left recv, all x values must be adjusted 1 cm.
#define Y_CORRECT   1 // [cm]   // servo horn is approx 10 mm forward of actual center of robot/us board.

#define M_PI    3.14159265358979323846264338327     // pi
#define RTD     57.295779513082320876798154814105   // multiply with radian to get degrees
#define DTR     0.01745329251994329576923690768489  // multiply with degree to get radians  

struct POSITION {
    float x;
    float y;
};

extern int ultrasonic_angle;

// function definitions

void ultrasonic_init(void);
void ultrasonic_trigger(void);
uint32_t ultrasonic_receive(cyhal_gpio_t echopin);  // This gets called as part of a distance lookup function
float ultrasonic_get_object_distance(cyhal_gpio_t echopin);  // Basic
struct * POSITION ultrasonic_locate_object();

#endif