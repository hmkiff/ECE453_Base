/*
 * ece453.h
 *
 *  Created on: November 3, 2023
 *      Author: Harry Kiffel
 */

#ifndef DRIVE_COMMANDS_H_
#define DRIVE_COMMANDS_H_


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <ctype.h>

#include "motor_functions.h"
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

void * charToMotor(char c);

void singleDrive(char name, int signal);

bool isMotorString(char * str);

void DriveMotor(struct MOTOR * motor, char * sig_str, int duty);

void DriveBot(char * sig_str, int duty);


//void DriveMotorA(char * sig_str, int duty);

#endif