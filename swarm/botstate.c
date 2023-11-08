#include "drivers/VL53L3CX_API_1.2.8/vl53lx_def.h"

struct botstate {

    // Stepper
    int step_speed;
    char step_dir;

    // IR
    VL53LX_MultiRangingData_t IR1;
    VL53LX_MultiRangingData_t IR2;
    VL53LX_MultiRangingData_t IR3;
    VL53LX_MultiRangingData_t IR4;

    // IMU

    // Ultrasonic
    uint32_t delay;
    float distInCm;
    float obj_ang_rad;
    float servo_ang_rad;

    // Encoders

};
