#ifndef BOTSTATE_H
#define BOTSTATE_H

#include "../drivers/VL53L3CX_API_1.2.8/vl53lx_def.h"
#include "../drivers/ir.h"

void print_botstate();

typedef struct {

    // IR
    VL53LX_MultiRangingData_t ir_data[NUM_IR];

    // These measurements should be relative to the bot,
    // where 0 rad is +x, or directly to the right of the bot.
    // IMU
    float imu_roll_rad;
    float imu_pitch_rad;
    float imu_yaw_rad;

    // Ultrasonic
    float servo_ang_rad;
    float us_echo1_cm;
    float us_echo2_cm;

} botstate;

#endif