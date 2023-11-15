#include "simbot.h"
#define M_PI 3.14159265358979323846

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

botstate simbot(botstate bot[NUM_BOTS], int simbot_ind, Environment env, SimbotBehavior behavior) {
    printf("Swarm: Updating simulated bot %i\r\n", simbot_ind);

    // Mock ultrasonic sweep
    if (behavior.sweep_us) {
        if (bot[simbot_ind].us_sweep_dir_bool == 0) {
            printf("Sweep right\n");
            if ((bot[simbot_ind].servo_ang_rad + behavior.sweep_us_step) > behavior.sweep_us_max) {
                bot[simbot_ind].servo_ang_rad = behavior.sweep_us_max;
                bot[simbot_ind].us_sweep_dir_bool = 1;
                printf("Sweep turn\n");
            } else {
                bot[simbot_ind].servo_ang_rad += behavior.sweep_us_step;
                printf("Sweep inc %f\n", behavior.sweep_us_step);
            }
        } else {
            printf("Sweep left\n");
            if ((bot[simbot_ind].servo_ang_rad - behavior.sweep_us_step) < behavior.sweep_us_min) {
                bot[simbot_ind].servo_ang_rad = behavior.sweep_us_min;
                bot[simbot_ind].us_sweep_dir_bool = 0;
                printf("Sweep turn\n");
            } else {
                bot[simbot_ind].servo_ang_rad -= behavior.sweep_us_step;
                printf("Sweep inc %f\n", behavior.sweep_us_step);
            }
        }
    }

    // Check line of sight
    // Check for walls
    
    // Check for other bots

    printf("Simbot: Bot %i sees object %fcm away at angle %f\r\n", simbot_ind, bot[simbot_ind].us_echo1_cm, bot[simbot_ind].servo_ang_rad);

    return bot[simbot_ind];
}