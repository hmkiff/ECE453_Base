#ifndef BOTSTATE_H
#define BOTSTATE_H

#include "../drivers/VL53L3CX_API_1.2.8/vl53lx_def.h"

typedef struct {
    float pos_x;
    float pos_y;
} botpos;

typedef struct {

    // Position
    botpos pos;

    // IR
    VL53LX_MultiRangingData_t ir1;
    VL53LX_MultiRangingData_t ir2;
    VL53LX_MultiRangingData_t ir3;
    VL53LX_MultiRangingData_t ir4;

    // IMU
    float imu_roll;
    float imu_pitch;
    float imu_yaw;

    // Ultrasonic
    float us_dist_cm;
    float us_servo_ang_rad;

    // Encoders

} botstate;

#endif