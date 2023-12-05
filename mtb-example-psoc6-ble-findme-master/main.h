/*
 * main.h
 *
 *  Created on: Aug 30, 2022
 *      Author: Joe Krachey
 */

#ifndef MAIN_H_
#define MAIN_H_

#define ECE453_USR_BTN P5_6
#define ECE453_USR_LED P5_5

#define ECE453_DEBUG_TX	P9_1
#define ECE453_DEBUG_RX P9_0

// Drivers
#include "drivers/leds.h"
#include "drivers/push_button.h"
#include "drivers/console.h"
#include "drivers/i2c.h"
#include "drivers/ir.h"
#include "drivers/ir_mux.h"
#include "drivers/eeprom.h"
#include "drivers/io-expander.h"
#include "drivers/LM75.h"
#include "drivers/VL53L3CX_API_1.2.8/vl53lx_api.h"
#include "drivers/motors/motor_functions.h"
#include "motion_control.h"
#include "drivers/ultrasonic.h"
#include "drivers/motors/drive_commands.h"

// Debug
#include "debug-tools/result_tools.h"

// Swarm
#include "swarm/botstate.h"
#include "swarm/swarm.h"

#endif /* MAIN_H_ */
