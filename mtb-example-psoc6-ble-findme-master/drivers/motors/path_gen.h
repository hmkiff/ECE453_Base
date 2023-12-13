// =============================================================================
// Adapted from code by Peter G. Adamczyk 
// 2018-10
// Updated 2021-02-27
// Adapted by: Harry K. Dec, 2023
// =============================================================================

#ifndef PATH_GEN_H_
#define PATH_GEN_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "closed_loop_path.h"
#include "vector_util.h"

#define QLENGTH 25
#define WAYPOINT_TOL 0.05

extern struct PATHSPEC path_queue[QLENGTH];
extern int pathlength;
extern int path_traveled;
extern struct POSE estimated_pose;
extern struct POSE target_pose;
extern struct POSE waypoints[QLENGTH];
extern bool waypoint_complete;


#endif

void createWaypointPath(struct POSE est_pose);

void waypointLoop();

void getEstPose();

struct POSE newPose();
// loops the path_specs array.
void pathLoop();

// updates target for bug
void refreshWaypoint();
