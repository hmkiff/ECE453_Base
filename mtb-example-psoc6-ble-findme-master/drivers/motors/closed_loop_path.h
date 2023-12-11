/*
 * Code adapted from closed_loop_path_follower.py 
 * =============================================================================
 * Peter G. Adamczyk 
 * 2018-10
 * Updated 2021-02-27
 * =============================================================================
 * 
 * Adapted By: Harry K. 12/4/2023
 */

#ifndef CLOSED_LOOP_PATH_H_
#define CLOSED_LOOP_PATH_H_

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "motion_control.h"
#include "vector_util.h"


// Get parameters from rosparam
// All you have when planning is a model - you never quite know the truth! 
#define BODY_LENGTH
#define WHL_DIAMETER
#define WHL_RADIUS


// Closed-loop controller parameters: get them from the ROS parameters imported from the YAML file. 
#define VMAX 0.0
#define BETA 0.0
#define GAMMA 0.0
#define ANGLE_FOCUS 0.0

// initial situation is that the path is NOT complete.
extern bool forward_only;

extern bool path_is_complete;


extern struct PATHSPEC path_segment_spec;
extern struct POSE estimated_pose_previous;

void set_path_complete(bool msg_in);

void update_path(struct PATHSPEC path_msg_in);

void path_follow(struct POSE * pose_msg_in);

#endif
