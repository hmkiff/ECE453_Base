#include "botstate.h"

void print_botstate(botstate state) {

    printf("------------\r\n");
    printf("Current pose\r\n");
    printf("------------\r\n");

    printf("- Bot pose theta: %f\r\n", state.bot_pose.theta);
    printf("- Bot pose x: %f\r\n", state.bot_pose.x);
    printf("- Bot pose y: %f\r\n", state.bot_pose.y);

    printf("----------------\r\n");
    printf("Destination pose\r\n");
    printf("----------------\r\n");

    printf("- Bot pose theta: %f\r\n", state.current_target.theta);
    printf("- Bot pose x: %f\r\n", state.current_target.x);
    printf("- Bot pose y: %f\r\n", state.current_target.y);

    printf("----------------\r\n");
    printf("Closest obstacle\r\n");
    printf("----------------\r\n");

    printf("- Obstacle x: %f\r\n", state.closest_obstacle.x);
    printf("- Obstacle y: %f\r\n", state.closest_obstacle.y);

    printf("--\r\n");
    printf("IR\r\n");
    printf("--\r\n");

    for (int i = 0; i < NUM_IR; i++) {
        VL53LX_MultiRangingData_t this_ir_data = state.ir_data[i];
        printf("- %i:\r\n", i);
        printf("\t- Closest: %i mm\r\n", this_ir_data.RangeData->RangeMinMilliMeter);
        printf("\t- Furthest: %i mm\r\n", this_ir_data.RangeData->RangeMaxMilliMeter);
        printf("\t- Number: %i objects\r\n", this_ir_data.NumberOfObjectsFound);
        printf("\t- Status: %i\r\n", this_ir_data.RangeData->RangeStatus);
    }

    printf("----------\r\n");
    printf("Ultrasonic\r\n");
    printf("----------\r\n");

    printf("- Servo angle: %f rad\r\n", state.servo_ang_rad);
    printf("- Echo 1: %f rad\r\n", state.us_echo1_cm);
    printf("- Echo 2: %f rad\r\n", state.us_echo2_cm);

}