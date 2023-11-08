#include "swarm.h"

botpos swarm(botstate bot[NUM_BOTS], int self_ind) {
    float x = bot[self_ind].pos.pos_x;
    float y = bot[self_ind].pos.pos_y;
    printf("Swarm: Bot %i was at (%f, %f)\r\n", self_ind, x, y);

    // Separate rule: try to avoid too much congestion with nearby partners.
    if (bot[self_ind].us_dist_cm < 15) {
        
    }
    
    // Cohesion rule: try to move towards the center of nearby partners.
    float x_sum = 0;
    float y_sum = 0;
    for (int i = 0; i < NUM_BOTS; i++) {
        x_sum += bot[i].pos.pos_x;
        y_sum += bot[i].pos.pos_y;
    }
    float x_center = x_sum / NUM_BOTS;
    float y_center = y_sum / NUM_BOTS;
    x += ((x_center - x) * COHESION_FACTOR);
    y += ((y_center - y) * COHESION_FACTOR);

    // Alignment rule: try to keep pace with average direction of nearby partners.

    // Naive test crawl
    botpos new_pos;
    new_pos.pos_x = x;
    new_pos.pos_y = y;
    printf("Swarm: Bot %i will go to (%f, %f)\r\n", self_ind, x, y);
    return new_pos;
}