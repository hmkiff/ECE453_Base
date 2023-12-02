/*
 * push_button.c
 *
 *  Created on: October 20, 2023
 *      Author: Harry Kiffel
 */

#include "ultrasonic.h"

int ultrasonic_angle = 0;

// 
void ultrasonic_init(void){
	
    // Config Trigger pin as output
    cyhal_gpio_init(
			PIN_TRIGGER,                // Pin
			CYHAL_GPIO_DIR_OUTPUT,      // Direction
			CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
			false);

    // Config Echo1 pin as input
    cyhal_gpio_init(
			PIN_ECHO1,                // Pin
			CYHAL_GPIO_DIR_INPUT,      // Direction
			CYHAL_GPIO_DRIVE_NONE,    // Drive Mode
			false);

    // Config Echo2 pin as input
    cyhal_gpio_init(
			PIN_ECHO2,                // Pin
			CYHAL_GPIO_DIR_INPUT,      // Direction
			CYHAL_GPIO_DRIVE_NONE,    // Drive Mode
			false);

}
void ultrasonic_trigger(void){

    // Start 10us trigger pulse
    cyhal_gpio_write(PIN_TRIGGER, true);
    printf("Trigger Start\r\n");
    // Delay 10us
    cyhal_system_delay_ms(10);	
    printf("Trigger Complete\r\n");
    // Stop Pulse
    cyhal_gpio_write(PIN_TRIGGER, false);
}

// Basic pulse interpreter for Echo pins
uint32_t ultrasonic_receive(cyhal_gpio_t echopin){
    printf("Start Receive\r\n");
    cy_rslt_t rslt;
    uint32_t read_val;
    /* Timer object used */
    cyhal_timer_t timer_obj;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = 200000,                    /* Timer period set to a large enough value
                                             * compared to event being measured */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = false,             /* Do not run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };
    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&timer_obj, NC, NULL);
    printf("Timer Initialized\r\n");
    /* Apply timer configuration such as period, count direction, run mode, etc. */
    cyhal_timer_configure(&timer_obj, &timer_cfg);
    /* Set the frequency of timer to 10000 counts in a second or 10000 Hz */
    cyhal_timer_set_frequency(&timer_obj, 1000000);
    printf("Pre-trigger\r\n");
    // trigger sensor signal
    ultrasonic_trigger();

    // wait until echo pulse starts
    while(cyhal_gpio_read(echopin) == 0);
    /* Start the timer with the configured settings */
    cyhal_timer_start(&timer_obj);
    printf("Timer Start");
    /* Read Echo PIN for High */ 
    while(cyhal_gpio_read(echopin) == 1);
    /* Read the current timer value, which should be close to the amount of delay in ms * 10 (5000) */
    read_val = cyhal_timer_read(&timer_obj);
    return read_val;
}

// Basic Distance capture for specified echo pin.
float ultrasonic_get_object_distance(cyhal_gpio_t echopin){
    uint32_t time_us = ultrasonic_receive(echopin);
    float cm_distance = 0.0343 * time_us;
    return cm_distance;
}


struct * POSITION ultrasonic_locate_object(){

    printf("Locating Object \r\n");

    // Ultrasonic Object Position Steps
    // Obtain both distances
    float leftDist = utrasonic_get_object_distance(PIN_ECHO1);
    float rightDist = ultrasonic_get_object_distance(PIN_ECHO2);
    printf("left: %f, right %f \r\n", leftDist, rightDist);

    float Llong  = 0;
    float Lshort = 0;
    bool bigPhi  = FALSE;

    // Compare which is longer
    if(leftDist == rightDist) { equalDist = TRUE;}
    else{
        if(rightDist > leftDist){
            Llong = rightDist;
            Lshort = leftDist;
            bigPhi = TRUE;
        }
        else{
            Llong = leftDist;
            Lshort = rightDist;
            bigPhi = FALSE;
        }
    }
    printf("short: %f, long %f \r\n", Lshort, Llong);
    
    float shortSquare = Lshort * Lshort;
    float longSquare = Llong * Llong;
    float deltaSquare = delta * delta;
    printf("shortSquare: %f, longSquare %f , deltaSquare: %f \r\n", shortSquare, longSquare, deltaSquare);

    // Solve for alpha (the angle between short distance and sensor board)
    // alpha = arccos( (Lshort^2 + delta^2 - Llong^2) / (2*Lshort*delta) )
    double alpha = acos((shortSquare + deltaSquare - longSquare) / (2*Lshort*delta));
    printf("alpha: %lf \r\n", alpha);

    // Solve for R, the radius from center of board (center of robot) to object
    // R = sqrt( Lshort^2 + (delta/2)^2 - 2*(delta/2)*Lshort*cos(alpha) )
    float Rdiff = 2*(delta/2)*Lshort*cos(alpha);
    float R = sqrt(shortSquare + (deltaSquare/4) - Rdiff);
    printf("sqrt(%f + %f - %f) = %f \r\n", shortSquare, (deltaSquare/4), Rdiff, R);
    
    //Solve for phi
    // phi = arcsin( (delta*sin(alpha)) / 2*R )
    double phi = asin( (delta*sin(alpha))/ 2*R );
    printf("phi: %lf \r\n", phi);
    double true_phi = (bigPhi ? (M_PI-phi) : phi);

    // Localize with position vector (R, phi)
    // Convert to X Y coordinates 
    float us_X = (R * cos(phi)) - X_CORRECT;
    float us_Y = (R * sin(phi)) - Y_CORRECT;
    printf("us_x: %f, us_y: %f \r\n", us_X, us_Y);
    
    // Change reference frame from sensor board frame to robot frame
    double us_angle_rad = (float) ultrasonic_angle * DTR;
    float true_X = us_X * cos(-us_angle_rad) - us_Y * sin(-us_angle_rad);
    float true_Y = us_X * sin(-us_angle_rad) + us_Y * cos(-us_angle_rad);

    struct POSITION outPos = {.x = true_X, .y = true_Y};
    printPosition(&outPos);
    
    return &outPos;


    
}

void printPosition(struct POSITION * pos){
    printf("x: %f, y: %f \r\n", pos.x, pos.y);
}