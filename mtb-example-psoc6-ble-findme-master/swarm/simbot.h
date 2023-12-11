/*
Abandoned.
#ifndef SIMBOT_H
#define SIMBOT_H

#include <math.h>
#include "botstate.h"
#include "environment.h"
#include "swarm.h"


typedef struct {
    int sweep_us;
    float sweep_us_step;
    float sweep_us_min;
    float sweep_us_max;
} SimbotBehavior;

botstate simbot_init(botstate bot[NUM_BOTS], int simbot_ind, SimbotBehavior behavior);
botstate simbot(botstate bot[NUM_BOTS], int simbot_ind, Environment env, SimbotBehavior behavior);

#endif
*/