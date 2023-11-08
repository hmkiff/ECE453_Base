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

#define NUM_BOTS 4
#define COHESION_FACTOR 0.2
#define SEPARATION_FACTOR 50
botpos swarm(botstate bot[NUM_BOTS], int self_ind);

#endif