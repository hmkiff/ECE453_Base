#ifndef BOTSTATE_H
#define BOTSTATE_H

#include "../drivers/VL53L3CX_API_1.2.8/vl53lx_def.h"

typedef struct {
    // These measurements should be relative to position at power on
    float pos_x;
    float pos_y;
    float heading_rad;
} botpos;

typedef struct {

    // Position
    botpos pos;

    // IR
    VL53LX_MultiRangingData_t ir1;
    VL53LX_MultiRangingData_t ir2;
    VL53LX_MultiRangingData_t ir3;
    VL53LX_MultiRangingData_t ir4;

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
    int us_sweep_dir_bool;

    // Encoders

} botstate;

#endif