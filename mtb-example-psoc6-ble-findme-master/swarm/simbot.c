/*
#include "simbot.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

botstate simbot_init(botstate bot[NUM_BOTS], int simbot_ind, SimbotBehavior behavior) {
    bot[simbot_ind].pos.pos_x = simbot_ind * 100 + 350;
    bot[simbot_ind].pos.pos_y = 100;
    bot[simbot_ind].pos.heading_rad = 0;
    bot[simbot_ind].servo_ang_rad = 0;
    bot[simbot_ind].us_echo1_cm = 10000;
    bot[simbot_ind].us_echo2_cm = 10000;
    bot[simbot_ind].us_sweep_dir_bool = 0;
    return bot[simbot_ind];
}

int circleLineTest(float cir_rad, float cir_x, float cir_y, 
    float pt_x, float pt_y, float look_ang) {

    // Is look angle in correct hemicircle?
    float cir_rel_x = cir_x - pt_x;
    float cir_rel_y = cir_y - pt_y;
    float cir_rel_ang = tan(cir_rel_y / cir_rel_x);
    if (abs(cir_rel_ang - look_ang) < M_PI) {
        float dist = sqrt(pow(cir_x - pt_x, 2) + pow(cir_y - pt_y, 2));
        if (abs(look_ang - cir_rel_ang) <= tan(cir_rad / dist)) {
            return dist - cir_rad;
        } else {
            return -1;
        }
    } else {
        return -1;
    }
}

botstate simbot(botstate bot[NUM_BOTS], int simbot_ind, Environment env, SimbotBehavior behavior) {
    printf("Swarm: Updating simulated bot %i\r\n", simbot_ind);

    // Mock ultrasonic sweep
    if (behavior.sweep_us) {
        if (bot[simbot_ind].us_sweep_dir_bool == 0) {
            if ((bot[simbot_ind].servo_ang_rad + behavior.sweep_us_step) > behavior.sweep_us_max) {
                bot[simbot_ind].servo_ang_rad = behavior.sweep_us_max;
                bot[simbot_ind].us_sweep_dir_bool = 1;
            } else {
                bot[simbot_ind].servo_ang_rad += behavior.sweep_us_step;
            }
        } else {
            printf("Sweep left\n");
            if ((bot[simbot_ind].servo_ang_rad - behavior.sweep_us_step) < behavior.sweep_us_min) {
                bot[simbot_ind].servo_ang_rad = behavior.sweep_us_min;
                bot[simbot_ind].us_sweep_dir_bool = 0;
            } else {
                bot[simbot_ind].servo_ang_rad -= behavior.sweep_us_step;
            }
        }
    }

    // Check line of sight
    // Check for other bots
    float closest = 100000;
    for (int i = 0; i < NUM_BOTS; i++) {
        if (i != simbot_ind) {
            int result = circleLineTest(50, bot[i].pos.pos_x, bot[i].pos.pos_y, 
                bot[simbot_ind].pos.pos_x, bot[simbot_ind].pos.pos_y, 
                bot[simbot_ind].servo_ang_rad + bot[simbot_ind].pos.heading_rad);
            if (result != -1) {
                if (result < closest) {
                    closest = result;
                }
            }
        }
    }
    bot[simbot_ind].us_echo1_cm = closest;
    bot[simbot_ind].us_echo2_cm = closest;

    // Check for walls

    printf("Simbot: Bot %i sees object %fcm away at angle %f\r\n", simbot_ind, bot[simbot_ind].us_echo1_cm, bot[simbot_ind].servo_ang_rad);

    return bot[simbot_ind];
}
*/