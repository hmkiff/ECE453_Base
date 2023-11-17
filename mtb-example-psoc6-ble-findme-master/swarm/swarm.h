// swarm.h
// To be called from main after subsystem initialization
//
// Refs:
//
// Improved artificial fish swarm algorithm
// https://ieeexplore.ieee.org/abstract/document/6931262

#ifndef SWARM_H
#define SWARM_H

#include <math.h>
#include "botstate.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NUM_BOTS 4

typedef enum {
    LINEAR,
    INVERSE,
    EXPONENTIAL,
    LOGARITHMIC
} Curve;

#define COHESION_FACTOR 0.1
#define COHESION_CURVE 1
#define SEPARATION_FACTOR 1
#define SEPARATION_CURVE 2
botpos swarm(botstate bot[NUM_BOTS], int self_ind);

#endif