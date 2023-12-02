/*
 * push_button.c
 *
 *  Created on: October 20, 2023
 *      Author: Harry Kiffel
 */

#include "ultrasonic.h"

int ultrasonic_angle = 0;
cyhal_pwm_t servo_pwm_obj;

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

    servo_init();

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
    printf("Timer Start\r\n");
    /* Read Echo PIN for High */ 
    int polls = 0;
    while((cyhal_gpio_read(echopin) == 1) && (polls < US_TIMEOUT)){polls++;}
    if(polls < US_TIMEOUT){
        /* Read the current timer value, which should be close to the amount of delay in ms * 10 (5000) */
        read_val = cyhal_timer_read(&timer_obj);
        return read_val;
    }
    else{
        printf("Ultrasonic Timeout\r\n");
        return -1;
    }
}

// Basic Distance capture for specified echo pin.
float ultrasonic_get_object_distance(cyhal_gpio_t echopin){
    uint32_t time_us = ultrasonic_receive(echopin);
    if(time_us > 0){
    float cm_distance = 0.0343 * time_us;
    return cm_distance;
    }
    else{
        printf("Invalid Ultrasonic Time\r\n");
        return -1;
    }
}


struct POSITION ultrasonic_locate_object(){

    printf("Locating Object \r\n");
    struct POSITION outPos = {.x = 0, .y = 0};
    // Ultrasonic Object Position Steps
    // Obtain both distances
    double leftDist = ultrasonic_get_object_distance(PIN_ECHO2);
    double rightDist = ultrasonic_get_object_distance(PIN_ECHO1);
    printf("left: %f, right %f \r\n", leftDist, rightDist);
    if((leftDist < 0) || (rightDist < 0)){
        return outPos;
    }
    double Llong     = 0;
    double Lshort    = 0;
    bool bigPhi     = false;
    bool equalDist  = false;

    // Compare which is longer
    if(leftDist == rightDist) { equalDist = true;}
    else{
        if(rightDist > leftDist){
            Llong = rightDist;
            Lshort = leftDist;
            bigPhi = true;
        }
        else{
            Llong = leftDist;
            Lshort = rightDist;
            bigPhi = false;
        }
    }
    printf("short: %f, long %f \r\n", Lshort, Llong);
    
    double shortSquare = Lshort * Lshort;
    double longSquare = Llong * Llong;
    double deltaSquare = DELTA * DELTA;
    printf("shortSquare: %lf, longSquare %lf , deltaSquare: %lf \r\n", shortSquare, longSquare, deltaSquare);

    // Solve for alpha (the angle between short distance and sensor board)
    // alpha = arccos( (Lshort^2 + delta^2 - Llong^2) / (2*Lshort*delta) )
    double Anumer = (shortSquare + deltaSquare - longSquare);
    double Adenom = (2*Lshort*DELTA);
    double alpha = acos(Anumer / Adenom);
    printf("Numerator: %lf, Denominator: %lf, Quotient: %lf \r\n", Anumer, Adenom, (Anumer/Adenom));
    printf("alpha: %lf rad, %lf deg\r\n", alpha, (alpha*RTD));

    // Solve for R, the radius from center of board (center of robot) to object
    // R = sqrt( Lshort^2 + (delta/2)^2 - 2*(delta/2)*Lshort*cos(alpha) )
    double Rdiff = 2*(DELTA/2)*Lshort*cos(alpha);
    double R = sqrt(shortSquare + (deltaSquare/4) - Rdiff);
    printf("sqrt(%f + %f - %f) = %f \r\n", shortSquare, (deltaSquare/4), Rdiff, R);
    
    //Solve for phi
    // phi = arcsin( (delta*sin(alpha)) / 2*R )
    double Pnumer = (Lshort*sin(alpha));
    double Pdenom =  R;
    double phi = asin(Pnumer / Pdenom);
    printf("Numerator: %lf, Denominator: %lf, Quotient: %lf \r\n", Pnumer, Pdenom, (Pnumer/Pdenom));
    printf("phi: %lf rad, %lf deg\r\n", phi, (phi*RTD));

    double true_phi = (bigPhi ? (M_PI-phi) : phi);
    printf("true_phi: %lf rad, %lf deg\r\n", true_phi, (true_phi*RTD));

    // Localize with position vector (R, phi)
    // Convert to X Y coordinates 
    float us_X = (equalDist ? 0: (R * cos(true_phi)) - X_CORRECT);
    float us_Y = (equalDist ? leftDist : (R * sin(true_phi)) - Y_CORRECT);
    printf("us_x: %f, us_y: %f \r\n", us_X, us_Y);
    
    int ultrasonic_frame = ultrasonic_angle - 90;
    // Change reference frame from sensor board frame to robot frame
    double us_angle_rad = (float) ultrasonic_frame * DTR;
    float true_X = us_X * cos(us_angle_rad) - us_Y * sin(us_angle_rad);
    float true_Y = us_X * sin(us_angle_rad) + us_Y * cos(us_angle_rad);

    outPos.x = true_X;
    outPos.y = true_Y;
    printPosition(&outPos);

    return outPos;
}

void printPosition(struct POSITION * pos){
    printf("x: %f, y: %f \r\n", pos->x, pos->y);
}






// Initializes the ultrasonic servo
void servo_init(void)
{
    // Enable SERVO_PWM output
	cyhal_pwm_init(
			&servo_pwm_obj, 
			PIN_SERVO_PWM, 
			NULL);
    set_servo_angle(90);
}

// set_servo_angle
void set_servo_angle(int angle){

	// stop signal briefly before changing it
	cyhal_pwm_stop(&servo_pwm_obj);

	// pulse width range 500us - 2500us 
	// neutral position (90 deg) 1500us
	if(angle < 0 || angle > 180){
		printf("CMD warning: Servo angle was out-of-bounds and rectified to 0 or 180\r\n");
        return;
	}
	// Calcullate pulse-width from angle
	int pulse_width = ((angle*2000)/180) + 500;
    //printf("angle: %d, pulse_width: %d, \r\n (angle*2000)/180: %d, (angle/180)*2000: %d, \r\n", angle, pulse_width, ((angle*2000)/180), ((angle/180)*2000));
	// Saturate width if out-of-bounds
	int set_width = pulse_width;
	if(pulse_width > 2500) set_width = 2500;
	if(pulse_width < 500) set_width = 500;
	//printf("Servo angle set to %d degrees (%d us)\r\n", angle, set_width);
    ultrasonic_angle = angle;
	cyhal_pwm_set_period(&servo_pwm_obj, 20000, set_width);
    cyhal_pwm_start(&servo_pwm_obj);

}

void set_servo_us(int width){
    cyhal_pwm_stop(&servo_pwm_obj);
    cyhal_pwm_set_period(&servo_pwm_obj, 20000, width);
    cyhal_pwm_start(&servo_pwm_obj);
}