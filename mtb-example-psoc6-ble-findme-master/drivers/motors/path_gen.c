// =============================================================================
// Adapted from code by Peter G. Adamczyk 
// 2018-10
// Updated 2021-02-27
// Adapted by: Harry K. Dec, 2023
// =============================================================================

#include "path_gen.h"

struct PATHSPEC path_queue[QLENGTH];
int pathlength; // length of planned path
int path_traveled = 0;
struct POSE estimated_pose;
struct POSE target_pose;
struct POSE waypoints[QLENGTH];
bool waypoint_complete = false;

void waypointLoop(){
// set up a rate basis to keep it on schedule.
    int index = 0;
    int waypoint_index;
    int updateDelay = 100;  // 100 ms 10Hz
    
    while(!path_is_complete){
        waypoint_index = (index % QLENGTH);
        target_pose = newPose();
        //IMU_read();
        createWaypointPath(estimated_pose);
        path_follow(estimated_pose);
        if(waypoint_complete){
            path_traveled++;
        }
        cyhal_system_delay_ms(updateDelay);
    }      
}

struct POSE newPose(){
    struct POSE outPose = estimated_pose;
    if(!path_is_complete){
        outPose.x       = 0;
        outPose.y       = 0;
        outPose.theta   = 0;//TODO Input XYTheta;
    } else{
        outPose = target_pose;
        path_is_complete = true;
        set_wheel_speeds(0,0);
    }
    return outPose;
}

// void pathLoop(){
// // set up a rate basis to keep it on schedule.
//     int index;
//     int specs_index = (index % QLENGTH);
//     int updateDelay = 100;  // 100 ms 10Hz
//     while(!path_is_complete){
//         est_pose = IMU_read();
//         createWaypointPath(est_pose);
//         path_follow(est_pose);
//         if(waypoint_complete){
//             path_traveled++;
//         }
//         if(path_traveled > pathlength){
//             return;
//         } else{
//             index++;
//         }
//         cyhal_system_delay_ms(updateDelay);
//     }     
// }

void createWaypointPath(struct POSE est_pose)
{
    struct PATHSPEC new_spec;
    //    CODE HERE: Change values of 0 to meaningful expressions. 
    // Find the X and Y distances from the current position 'estimated_pose' to the active waypoint 'waypoint'
    double dx = target_pose.x - est_pose.x;     // distance in X coords
    double dy = target_pose.y - est_pose.y;    // distance in Y coords
    
    // A straight line directly from the current location to the intended location. 
    new_spec.x0 = est_pose.x;   // Current Location x
    new_spec.y0 = est_pose.y;    // Current Location y
    new_spec.theta0 = atan2f(-dx, dy);   // Angle to the endpoint, using the customary y-forward coordinates. Use arctan2. 
    new_spec.Radius = INFINITY;    // Radius of a Straight Line
    new_spec.Length = sqrt((dx*dx) + (dy*dy));    // Distance to the endpoint
//    CODE END    
    
    if(!isnan(path_segment_spec.Length)){
        path_segment_spec = new_spec;
    }
    // Check if the robot has arried at the waypoint 
    //  (i.e., if the path Length to waypoint is shorter than 'waypoint_tolerance'). 
    // If so, publish "waypoint_complete" with value True exactly once. 
    if (((path_segment_spec.Length < WAYPOINT_TOL)) && !waypoint_complete){ // DO NOT send more than once. Wait for a new path segment before sending "waypoint_complete" again 
        waypoint_complete = true;
    }

}
  