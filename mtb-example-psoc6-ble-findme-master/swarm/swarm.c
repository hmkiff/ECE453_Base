#include "swarm.h"

float curve(float val, Curve type, float factor) {
    if (type == INVERSE) {
        if (val > 0.02) {
            val = (1 / val);
        }
    } else if (type == EXPONENTIAL) {
        val = exp(val);
    } else if (type == LOGARITHMIC) {
        val = log(val);
    }
    return val * factor;
}

/*
botpos swarm(botstate bot[NUM_BOTS], int self_ind) {
    float x = bot[self_ind].pos.pos_x;
    float y = bot[self_ind].pos.pos_y;
    printf("Swarm: Bot %i was at (%f, %f)\r\n", self_ind, x, y);

    // Separate rule: try to avoid too much congestion with nearby partners.
    float avg_dist = (bot[self_ind].us_echo2_cm + bot[self_ind].us_echo1_cm) / 2;
    float factor = curve(avg_dist, SEPARATION_CURVE, SEPARATION_FACTOR);
    float separation_x = factor * sin(bot[self_ind].servo_ang_rad + M_PI);
    float separation_y = factor * cos(bot[self_ind].servo_ang_rad + M_PI);
    x += separation_x;
    y += separation_y;
    printf("Swarm: Separation influence: (%f, %f)\r\n", separation_x, separation_y);

    // Cohesion rule: try to move towards the center of nearby partners.
    float x_sum = 0;
    float y_sum = 0;
    for (int i = 0; i < NUM_BOTS; i++) {
        x_sum += bot[i].pos.pos_x;
        y_sum += bot[i].pos.pos_y;
    }
    float x_center = x_sum / NUM_BOTS;
    float y_center = y_sum / NUM_BOTS;
    float cohesion_x = curve(abs(x_center - x), COHESION_CURVE, COHESION_FACTOR);
    float cohesion_y = curve(abs(y_center - y), COHESION_CURVE, COHESION_FACTOR);
    printf("Swarm: Cohesion influence: (%f, %f) due to center at (%f, %f)\r\n", cohesion_x, cohesion_y, x_center, y_center);

    // Alignment rule: try to keep pace with average direction of nearby partners.

    botpos new_pos;
    new_pos.heading_rad = tan((bot[self_ind].pos.pos_y - y) / (bot[self_ind].pos.pos_x - x));
    new_pos.pos_x = x;
    new_pos.pos_y = y;
    printf("Swarm: Bot %i will go to (%f, %f)\r\n", self_ind, x, y);
    return new_pos;
}
*/