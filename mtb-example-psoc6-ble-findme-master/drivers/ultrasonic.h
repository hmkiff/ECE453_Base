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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

// function definitions

#define PIN_TRIGGER P9_3
#define PIN_ECHO1   P10_1 // P9_1 // 
#define PIN_ECHO2   P10_5 // P9_0 // 

void ultrasonic_init(void);
void ultrasonic_trigger(void);
uint32_t ultrasonic_receive(cyhal_gpio_t echopin);  // This gets called as part of a distance lookup function
float ultrasonic_get_object_distance(cyhal_gpio_t echopin);  // Basic

struct POSITION {
    uint32 x;
    uint32 y;
};


#endif