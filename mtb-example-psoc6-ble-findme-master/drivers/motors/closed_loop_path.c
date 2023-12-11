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

#include "closed_loop_path.h"

// global variable to hold the path specs currently being tracked
struct PATHSPEC path_segment_spec = {.x0=0.0, .y0=0.0, .theta0=0.0, .Radius=INFINITY, .Length=INFINITY};
    
// global variables used in path following: 
bool initialize_psi_world= true;
struct POSE estimated_pose_previous = {.x=0.0, .y=0.0, .theta=0.0};
double estimated_x_local_previous = 0.;
double estimated_theta_local_previous= 0. ;
double path_segment_curvature_previous = 0.;
double estimated_psi_world_previous = 0. ;
double estimated_segment_completion_fraction_previous = 0.;


// =============================================================================
// // Function to tell the path segment specifier node whether the path is finished
// =============================================================================
void set_path_complete(bool msg_in){
    path_is_complete = msg_in;
}

// =============================================================================
// // Function to update the path segment currently being tracked
// =============================================================================
void update_path(struct PATHSPEC path_msg_in){
    path_segment_spec = path_msg_in;
}

// =============================================================================
// // Function to update the path tracking control based on the robot's estimated position
// =============================================================================
void path_follow(struct POSE pose_msg_in){
    // First assign the incoming message
    struct POSE estimated_pose = pose_msg_in;
    struct VEC2D pose_xy = {estimated_pose.x, estimated_pose.y};

    double estimated_y_local;
    double estimated_x_local;
    double estimated_theta_local;

    double estimated_segment_forward_pos;
    double estimated_segment_completion_fraction;

    float pose_theta = {estimated_pose.theta};
    if(isinf(path_segment_spec.Length)){
        return;
    }

    // Create a vector variable for the Origin of the path segment 
    struct VEC2D path_segment_Origin = {path_segment_spec.x0, path_segment_spec.y0};

    // Forward distance relative to a Line path is computed along the Forward axis. 
    // Therefore Define the Forward Axis: 
    struct VEC2D path_segment_y0_vector = {-sin(path_segment_spec.theta0), cos(path_segment_spec.theta0)};
    // local X is computed perpendicular to the segment. 
    // Therefore define the Perpendicular axis.
    struct VEC2D path_segment_x0_vector = {-sin(path_segment_spec.theta0 -(M_PI/2.0)), cos(path_segment_spec.theta0 - (M_PI/2.0))};
    // Define path curvature
    double path_segment_curvature = 1.0/path_segment_spec.Radius;
    
    
    // =============================================================================
    //     // First Compute the robot's position relative to the path (x, y, theta)
    //     // and the local path properties (curvature 1/R and segment completion percentage)
    // =============================================================================
        
    // If it is a Line (Infinite radius)
    if (isinf(path_segment_spec.Radius)){
        struct VEC2D path_segment_endpoint = addVec(path_segment_Origin, scalarMultVec(path_segment_y0_vector, path_segment_spec.Length));
        
        // compute position relative to Segment: 
        struct VEC2D estimated_xy_rel_to_segment_origin = subVec(pose_xy, path_segment_Origin);   // XY vector from Origin of segment to current location of robot. 
        // Projection of vector from path origin to current position 
        estimated_segment_forward_pos = dot2DVec(estimated_xy_rel_to_segment_origin,path_segment_y0_vector);
        // The fraction completion can be estimated as the path length the robot has gone through, as a fraction of total path length on this segment
        estimated_segment_completion_fraction = estimated_segment_forward_pos / path_segment_spec.Length;
        // Position of the robot to the Right of the segment Origin
        double estimated_segment_rightward_pos = dot2DVec(estimated_xy_rel_to_segment_origin,path_segment_x0_vector);
        
        estimated_y_local = 0.0;    // y_local = 0 by definition: Local coords are defined relative to the closest point on the path. 
        estimated_x_local = estimated_segment_rightward_pos;
        estimated_theta_local = pose_theta - path_segment_spec.theta0;
    
    // Arc
    } else {
        double curve_sign = signf(path_segment_spec.Radius);
        struct VEC2D path_segment_circle_center = addVec(path_segment_Origin, scalarMultVec(path_segment_x0_vector,-(path_segment_spec.Radius)));
        // determine the angular displacement of this arc. SIGNED quantity! 
        double path_segment_angular_displacement = path_segment_spec.Length/path_segment_spec.Radius;
        double path_segment_ThetaEnd = path_segment_spec.theta0 + path_segment_angular_displacement;
        struct VEC2D estimated_xy_rel_to_circle_center = subVec(pose_xy, path_segment_circle_center);
        
        // Compute angle of a vector from circle center to Robot, in the world frame, relative to the +Yworld axis. 
        // Note how this definition affects the signs of the arguments to "arctan2"
        double estimated_psi_world = atan2f(-estimated_xy_rel_to_circle_center.i, estimated_xy_rel_to_circle_center.j);
        // unwrap the angular displacement
        if(initialize_psi_world){
            estimated_psi_world_previous = estimated_psi_world;
            initialize_psi_world = false;
        }
        while(estimated_psi_world - estimated_psi_world_previous > M_PI){ // was negative, is now positive --> should be more negative. 
            estimated_psi_world += -2.0*M_PI;
        }
        while(estimated_psi_world - estimated_psi_world_previous < -M_PI){ // was positive and is now negative --> should be more positive. 
            estimated_psi_world += 2.0*M_PI;
        }

        // update the "previous angle" memory. 
        estimated_psi_world_previous = estimated_psi_world;
        // The local path forward direction is perpendicular (clockwise) to this World frame origin-to-robot angle. 
        double estimated_path_theta = estimated_psi_world + ((M_PI/2.0)*curve_sign);
        // The fraction completion can be estimated as the path angle the robot has gone through, as a fraction of total angular displacement on this segment
        estimated_segment_completion_fraction = (estimated_path_theta - path_segment_spec.theta0) / path_segment_angular_displacement;

        estimated_y_local = 0.0;  // by definition of local coords
        // x_local is positive Inside the circle for Right turns, and Outside the circle for Left turns
        estimated_x_local = curve_sign*(sqrt(sumVec(squareVec(estimated_xy_rel_to_circle_center))) - fabsf(path_segment_spec.Radius) ) ;
        estimated_theta_local = pose_theta - estimated_path_theta;
    }
    // Whether Line or Arc, update the "local" coordinate state and path properties: 
    estimated_theta_local = fix_angle_pi_to_neg_pi(estimated_theta_local);

    // Update the "previous" values 
    estimated_pose_previous = estimated_pose;
    estimated_x_local_previous = estimated_x_local;
    estimated_theta_local_previous = estimated_theta_local;
    path_segment_curvature_previous = path_segment_curvature;
    estimated_segment_completion_fraction_previous = estimated_segment_completion_fraction;  


    // =============================================================================
    //     // CONTROLLER for path tracking based on local position and curvature. 
    //     // parameters for the controller are 
    //     //   VMAX: Maximum allowable speed,  
    //     // and controller gains:
    //     //   BETA (gain on lateral error, mapping to lateral speed)
    //     //   GAMMA (gain on heading error, mapping to rotational speed)
    //     // and a control variable for the precision of turns, 
    //     //   ANGLE_FOCUS
    // =============================================================================

    //    CODE HERE:  Put in formulas for anything that is 0.0, and try the "TRY THIS" variations. 
    // First set the speed with which we want the robot to approach the path
    double xdot_local_desired = -BETA*estimated_x_local;   // Use formula from Lecture
    // limit it to +-VMAX
    xdot_local_desired = fmin(fabsf(xdot_local_desired),fabsf(VMAX))*signf(xdot_local_desired);
    
    // Next set the desired theta_local 
    double theta_local_desired = asin((-xdot_local_desired)/VMAX);   // Use formula from Lecture
            
    // Next SET SPEED OF ROBOT CENTER. 
    // G. Cook 2011 says just use constant speed all the time,
    // TRY THIS FIRST
    double Vc = VMAX;    

    // But, that drives farther from the path at first if it is facing away. 
    // This FIX causes the speed to fall to zero if the robot is more than 90 deg from the heading we want it to have. 
    //   The parameter "ANGLE_FOCUS" can make it even more restrictive if needed (e.g. ANGLE_FOCUS = 2 --> 45 deg limit). 
    //   Value of 1.0 uses a straight cosine of the angle. 
    // TRY WITH AND WITHOUT THIS 
    Vc = VMAX * cos(ANGLE_FOCUS * fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local));
    
    // Could also limit it to only forward. 
    // TRY WITH AND WITHOUT THIS 
    if(forward_only){ 
        Vc = fmax(Vc,0);  // equivalently: Vc = np.max([Vc,Vc*int(forward_only)])
    }
    // Finally set the desired angular rate
    double estimated_theta_local_error = fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local);
    double K = path_segment_curvature; // for ease of writing
    
    double g_factor = (GAMMA*(theta_local_desired - estimated_theta_local));
    
    double speed_factor = ((K/(1+(K*estimated_x_local)))*Vc*cos(estimated_theta_local));

    double omega = g_factor + speed_factor;   
    
    // Finally, use the "robot" object created elsewhere (member of the me439_mobile_robot_xx class) to translate omega and Vc into wheel speeds
    set_wheel_speeds_from_robot_velocities(Vc, omega);
    
    // Check if the overall path is complete 
    // If so, stop!    
    if(path_is_complete){
        set_wheel_speeds(0.,0.);
        return;
    }   
     
    printf("Fraction of Segment Complete: %lf \r\n",estimated_segment_completion_fraction);

}