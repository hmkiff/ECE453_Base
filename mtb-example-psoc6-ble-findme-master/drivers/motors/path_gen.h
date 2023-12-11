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
#define WAYPOINT_TOL 1.0

extern struct PATHSPEC * path_queue[QLENGTH];
extern int pathlength;
extern struct POSE * est_pose;
extern struct POSE * target_pose;
bool waypoint_complete = false;






#endif

void createWaypointPath(struct POSE * est_pose);

void waypointLoop();

struct POSE newPose();
// loops the path_specs array.
void pathLoop();

// updates target for bug
void refreshWaypoint();
