#ifndef BOTSTATE_H
#define BOTSTATE_H

#include "../drivers/VL53L3CX_API_1.2.8/vl53lx_def.h"
#include "../drivers/ir.h"
#include "../drivers/motors/path_gen.h"
#include "../drivers/ultrasonic.h"

void print_botstate();

typedef struct {

    // Motion
    // three doubles : x, y, theta (rads)  this will hold the estimated position of the bot.
    struct POSE bot_pose;  
    // same struct but this will hold the current target_pose of the bot. 
    struct POSE current_target;

    // IR
    VL53LX_MultiRangingData_t ir_data[NUM_IR];

    // Ultrasonic
    float servo_ang_rad;
    float us_echo1_cm;
    float us_echo2_cm;

    // struct of two floats: x and y currently in cm, but units can change later.
    struct POSITION closest_obstacle;

} botstate;

#endif