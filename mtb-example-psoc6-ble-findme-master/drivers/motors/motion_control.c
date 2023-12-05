/*
 * motion_control.c
 *
 *  Created on: Dec 4, 2023
 *      Author: Harry Kiffel
 */

#include "motion_control.h"


struct ROBOT bug = {
    .r_center_world = {.i=0.0, .j=0.0}, 
    .v_center_world = {.i=0.0, .j=0.0}, 
    .theta = 0.0, .omega = 0.0, 
    .left_wheel_speed = 0.0, 
    .right_wheel_speed = 0.0,
    .left_wheel_distance_traveled = 0.0,
    .right_wheel_distance_traveled = 0.0,
    .left_wheel_angle = 0.0,
    .right_wheel_angle = 0.0
};

// Create a rotation matrix to translate the robot frame to the world-frame
struct MAT2 get_rotmat_body_to_world(){
        struct MAT2 rotmat_body_to_world = {.a=cos(bug.theta), .b=-sin(bug.theta), .c=sin(bug.theta), .d=cos(bug.theta)};
        return rotmat_body_to_world;
}

// update the wheel speeds of the robot struct
void set_wheel_speeds(double v_left, double v_right){
        
        bug.left_wheel_speed = v_left;
        bug.right_wheel_speed = v_right;
        set_motor_speed_mps(&motorA, v_left);
        set_motor_speed_mps(&motorB, v_right);
        drive_update();

        // Now compute the new robot velocities based on the new wheel_speeds. 
        // put in Equations (i) and (ii) here based on bug.left_wheel_speed, bug.right_wheel_speed, and bug.wheel_width
        double v_center_y_bf = (v_left + v_right)/2.0;
        struct VEC2D v_center_bf = {0.0, v_center_y_bf};
        double omega = (v_right - v_left)/WHEEL_WIDTH;
        bug.omega = omega;
        
        // Equation vi: rotation matrix
        struct MAT2 rotbf2w = get_rotmat_body_to_world();
        
        // Rotate velocity into world frame - Equation viii  (rotmat * v_center)
        bug.v_center_world = rotVec( rotbf2w,  v_center_bf);
}    

// update wheel speeds in robot struct based on robot velocities
void set_wheel_speeds_from_robot_velocities(double forward_velocity, double angular_velocity){
        // Kinematic Equations mapping desired robot speed and angular velocity
        // to left and right wheel speeds. 
        // Use the variables "forward_velocity" and "angular_velocity" that the function takes as input. 
        // Use "bug.wheel_width" as the robot's parameter for wheel width. 
        double left_wheel_speed  = forward_velocity - (angular_velocity*WHEEL_WIDTH/2);
        double right_wheel_speed = forward_velocity + (angular_velocity*WHEEL_WIDTH/2);
        set_wheel_speeds(left_wheel_speed, right_wheel_speed);
}

// Returns adjusted angle from pi to negative pi;
float fix_angle_pi_to_neg_pi(float angle){
    // Function to "unwrap" angles into the [-pi, pi) interval
    return fmod((angle+M_PI), (2*M_PI)) - M_PI;      
}


// =============================================================================
//      Plan a line: accept input of speed (m/s) and line segment length (m). 
//      Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]
// =============================================================================
struct WHL_CMD plan_line( float speed, float length){
    // Trick: correct sign mismatches between speed and length
    float new_speed = fabsf(speed)*signf(length);
    
    // Compute the needed wheel speeds to go straight: 
    float lft_whl_spd = new_speed;
    float rgt_whl_spd = new_speed;
    
    // COMPUTE the duration (time in seconds) for which they should turn at these speeds.
    float duration = length/speed;
    
    struct WHL_CMD line = {.duration = duration, .lspeed = lft_whl_spd, .rspeed = rgt_whl_spd};
    return line;
}

// =============================================================================
//     // Plan an Arc: accept input of arc radius (m), speed (m/s), and angle around the circle (RADIANS)
//     // ** NOTE the radius and angle both have Signs: Left turns are Positive, Right turns Negative
//     // Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]
// =============================================================================
struct WHL_CMD plan_arc(float radius, float speed, float angle){
    // COMPUTE the Left and Right wheel speeds using formulas from lecture
    float lft_whl_spd = speed*(1 - (WHEEL_WIDTH/(2*radius) ) );
    float rgt_whl_spd = speed*(1 + (WHEEL_WIDTH/(2*radius) ) );
    
    // COMPUTE "omega" - the angular rate of progress around the circle
    float omega = speed/radius;
    
    // Trick: correct funky combinations of signs
    omega = fabsf(omega)*signf(angle);
    
    // COMPUTE the duration (time in seconds) for which they should turn at these speeds.
    float duration = angle/omega;
    
    struct WHL_CMD arc = {.duration = duration, .lspeed = lft_whl_spd, .rspeed = rgt_whl_spd};
    return arc;
}
        
// =============================================================================
//     // Plan a Pivot: accept angular velocity (rad/s) and angle to turn (rad)
//     // ** NOTE the angle and angular velocity both have signs by the Right Hand Rule: 
//     //    Counterclockwise: positive; Clockwise: negative
//     // Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]    
// =============================================================================
struct WHL_CMD plan_pivot(float omega, float angle){
    // arguments: angular velocity (rad/sec), angle to turn (rad)
    // Trick: correct sign mismatches between omega and angle
    float new_omega = fabsf(omega)*signf(angle);
    
    // COMPUTE wheel speeds using equations from lecture 
    float radius = 0; // for a Pivot
    float lft_whl_spd = new_omega*(radius-(WHEEL_WIDTH/2));
    float rgt_whl_spd = new_omega*(radius+(WHEEL_WIDTH/2));
    
    // COMPUTE the duration (time in seconds) for which they should turn at these speeds.
    float duration = angle/new_omega;
    
    struct WHL_CMD pivot = {.duration = duration, .lspeed = lft_whl_spd, .rspeed = rgt_whl_spd};
    return pivot;
}      

// =============================================================================
//     // One more: allow the robot to sit still for a specified time (s)
//     // Return [duration (s), left_wheel_speed (m/s), right_wheel_speed (m/s)]      
// =============================================================================
struct WHL_CMD plan_pause(float pause_time){
    float lft_whl_spd = 0.;
    float rgt_whl_spd = 0.;
    float duration = pause_time;
    
    struct WHL_CMD pause = {.duration = duration, .lspeed = lft_whl_spd, .rspeed = rgt_whl_spd};
    return pause;       
}

// specifies a line from point A to point B
struct PATHSPEC specify_line(float x0, float y0, float xf, float yf){
    // Function to create specs for a line from [x0,y0] to [xf,yf]. 
    // arguments: x0 (m), y0 (m), xfinal (m), yfinal (0)

    float vec[2] = {xf-x0,yf-y0};                           // vector from [x0,y0] to [xf,yf], stored as a numpy array (np.array())
    float dist = sqrt((vec[0]*vec[0]) + (vec[1]*vec[1]));  // distance from [x0,y0] to [xf,yf], a Scalar
    float vecangle = atan2f(vec[1],vec[0]);                 // Vector angle: angle of the vector directly to the end point. A Scalar, compute with numpy's "arctan2"
    struct PATHSPEC specs = {.x0=x0, .y0=y0, .theta0=vecangle, .Radius=INFINITY, .Length=dist};
    return specs;
}

// Sepcifies the path for an arc from point A to point B
struct PATHSPEC specify_arc(float x0, float y0, float xf, float yf, float R, bool way){
    // Function to create specs for an arc from [x0,y0] to [xf,yf] with radius R, 
    // going the "short way" or the "long way" around the circle.
    // arguments: x0 (m), y0 (m), xfinal (m), yfinal (0), Radius (m), short way: 1 long way: 0    
    float x_vec = xf-x0;
    float y_vec = yf-y0;
    float vec[2] = {x_vec,y_vec};                        // vector from [x0,y0] to [xf,yf], stored as a numpy array (np.array())
    float dist = sqrt((x_vec*x_vec) + (y_vec*y_vec));   // distance from [x0,y0] to [xf,yf], a Scalar
    struct PATHSPEC specs;
    
    // Code to determine how a circular path of Radius R can get from start to finish of this path segment.  
    if(dist > 2.0*fabsf(R)){                 // If the endpoint is farther away than the diameter of the circle, an arc cannot get there and you must draw a line instead. 
        struct PATHSPEC line_specs = specify_line(x0, y0, xf,yf); // Use the "bug.specify_line(...)" function above for this case! 
        specs = line_specs;
    } else{  //Otherwise find an arc. 
        float arcangle = 2.0*asin((dist/2.0)/R );  // Smallest arc angle is swept if going the "short way" around the circle. The distance from start to finish is the "chord length" and relates to the included angle as shown. 
        if(way == 0){
            arcangle = 2*M_PI*signf(arcangle) - arcangle;    // arc angle swept if going the "long way": a full circle is 2*pi, so the long way is 2*pi minus the short way. Note the signs. 
        }
        float vecangle = atan2f(y_vec,x_vec);           // Vector angle: angle of the vector directly from the start point to the end point.
        float initangle = vecangle - (arcangle/2.0);    // Initial tangent angle of the arc to be followed. Deviates from the direct straignt line by half the included angle of the arc. 
        initangle = fix_angle_pi_to_neg_pi(initangle);  // Could have gone past pi with the addition. Unwrap to make sure it's within +/- pi. 
        float arclength = R*vecangle;                   // arc length is also needed. Enter it based on Radius and Angle. 
        struct PATHSPEC arc_specs = {.x0=x0, .y0=y0, .theta0=initangle, .Radius=R, .Length=arclength};
        specs = arc_specs;
    }
    
    return specs;	
}


//  void drive_line(int distance_cm, float speed_mps){
// 	int duty = speed_mps * MPStoDC;
// 	set_drive_direction(1);
// 	set_drive_duty(duty);
// 	drive_update();
// 	cyhal_system_delay_ms((distance_cm*1000)/speed_mps);
// 	set_drive_duty(0);
// 	set_drive_direction(0);
// 	drive_update();
// 	printf("Line Complete\r\n");
// }

// void drive_arc(float turn_radius, float speed_mps, int direction){
// 	int duty = speed_mps * MPStoDC;
// 	set_drive_direction(FORWARD);
// 	int inner_rad = (turn_radius - (WHEEL_WIDTH/2));
// 	int outer_rad = (turn_radius + (WHEEL_WIDTH/2));
// 	int speed_left = 0;
// 	int speed_right = 0;
// 	set_motor_duty(&motorA, speed_left);
// 	set_motor_duty(&motorB, speed_right);
// 	drive_update();
// }