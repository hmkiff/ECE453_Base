/*
 * push_button.c
 *
 *  Created on: October 9, 2023
 *      Author: Harry Kiffel
 */

#include "motor_functions.h"

bool DIR = 0;	// Direction of STEP, 0: Left, 1: Right
cy_rslt_t   rslt;
cyhal_pwm_t step_pwm_obj;
cyhal_pwm_t servo_pwm_obj;

// drive motor pwms
cyhal_pwm_t drive1A_pwm_obj;
cyhal_pwm_t drive2A_pwm_obj;
cyhal_pwm_t drive1B_pwm_obj;
cyhal_pwm_t drive2B_pwm_obj;

struct MOTOR motorA = {.name = 'a', .motor_pwm[0] = &drive1A_pwm_obj, .motor_pwm[1] = &drive2A_pwm_obj, .duty = 0, .direction = 0, .sig1active = 0, .sig2active = 0};
struct MOTOR motorB = {.name = 'b', .motor_pwm[0] = &drive1B_pwm_obj, .motor_pwm[1] = &drive2B_pwm_obj, .duty = 0, .direction = 0, .sig1active = 0, .sig2active = 0};

// Drive motor functions

void drive_motor_init(void)
{
	// init drive motor A pwms
	cyhal_pwm_init( &drive1A_pwm_obj, PIN_MOTOR_1A, NULL);
	cyhal_pwm_init( &drive2A_pwm_obj, PIN_MOTOR_2A, NULL);
	// init drive motor B pwms
	cyhal_pwm_init( &drive1B_pwm_obj, PIN_MOTOR_1B, NULL);
	cyhal_pwm_init( &drive2B_pwm_obj, PIN_MOTOR_2B, NULL);

	// Set a duty cycle of 0% (coast) and frequency of 20Hz
	// motorA
    cyhal_pwm_set_duty_cycle(&drive1A_pwm_obj, 0, DRV_PWM_FREQ);
	cyhal_pwm_set_duty_cycle(&drive2A_pwm_obj, 0, DRV_PWM_FREQ);
	// motorB
	cyhal_pwm_set_duty_cycle(&drive1B_pwm_obj, 0, DRV_PWM_FREQ);
	cyhal_pwm_set_duty_cycle(&drive2B_pwm_obj, 0, DRV_PWM_FREQ);

}

void set_drive_motor_signal(struct MOTOR *motor, int signal, int duty)
{

	// obtain reference to motor's pwm signal.
	int signal_index = signal - 1;
	cyhal_pwm_t * drive_pwm_obj = motor->motor_pwm[signal_index];
	printf("Name: %c, Signal: %d, Duty %d \r\n", motor->name, signal_index, duty);

	// stop signal briefly before changing it
	cyhal_pwm_stop(drive_pwm_obj);

	// set duty cycle to duty
	cyhal_pwm_set_duty_cycle(drive_pwm_obj, duty, DRV_PWM_FREQ);
	
	// Start the PWM output
    cyhal_pwm_start(drive_pwm_obj);
}

void kill_motor_signal(){
	cyhal_pwm_stop(motorA.motor_pwm[0]);
	cyhal_pwm_stop(motorA.motor_pwm[0]);
	cyhal_pwm_stop(motorB.motor_pwm[0]);
	cyhal_pwm_stop(motorB.motor_pwm[0]);
}

void set_motor_direction(struct MOTOR *motor, int direction){
	motor->direction = direction;

	// counter-clockwise
	if(direction > 0){
		motor->sig1active = 1;
		motor->sig2active = 0;
	}
	// clockwise
	else if(direction < 0){
		motor->sig1active = 0;
		motor->sig2active = 1;
	}
	// brake
	else if(direction = 0){
		motor->sig1active = 0;
		motor->sig2active = 0;
	}
}

void set_motor_duty(struct MOTOR *motor, int duty){
	motor->duty = duty;
}

void set_drive_direction(int direction){
	
	// forward
	if(direction > 0){
		set_motor_direction(&motorA, 1);
		set_motor_direction(&motorB, 0);
	}
	// reverse
	else if(direction < 0){
		set_motor_direction(&motorA, 0);
		set_motor_direction(&motorB, 1);
	}
	// brake
	else if(direction = 0){
		set_motor_direction(&motorA, 0);
		set_motor_direction(&motorB, 0);
	}
}

void set_drive_duty(int duty){
	set_motor_duty(&motorA, duty);
	set_motor_duty(&motorB, duty);
}

void drive_update(){
	set_drive_motor_signal(&motorA, 1, motorA.sig1active * motorA.duty);
	set_drive_motor_signal(&motorA, 2, motorA.sig2active * motorA.duty);
	set_drive_motor_signal(&motorA, 1, motorB.sig1active * motorA.duty);
	set_drive_motor_signal(&motorA, 2, motorB.sig2active * motorA.duty);
}








void drive_line(int distance_cm, float speed_mps){
	int duty = speed_mps * MPStoDC;
	set_drive_direction(1);
	set_drive_duty(duty);
	drive_update();
	cyhal_system_delay_ms((distance_cm*1000)/speed_mps);
	set_drive_duty(0);
	set_drive_direction(0);
	drive_update();
	printf("Line Complete\r\n");
}

void drive_arc(float turn_radius, float speed_mps, int direction){
	int duty = speed_mps * MPStoDC;
	set_drive_direction(FORWARD);
	int inner_rad = (turn_radius - (WHEEL_WIDTH/2));
	int outer_rad = (turn_radius + (WHEEL_WIDTH/2));
	int speed_left = 0;
	int speed_right = 0;
	set_motor_duty(&motorA, speed_left);
	set_motor_duty(&motorB, speed_right);
	drive_update();
}

void print_motor(struct MOTOR * motor){
	printf("=================================\r\n");
	printf("Motor Name: %c\r\n", motor->name);
	printf("Duty: %d, Dir: %d, sig1active: %d, sig2active: %d\r\n", motor->duty, motor->direction, motor->sig1active, motor->sig2active);
	printf("=================================\r\n");
}




// Generic functions

//void motorfree(){
//
//	cyhal_pwm_free(&step_pwm_obj);
//	cyhal_pwm_free(&servo_pwm_obj);
//
//}
/*****************************************************
* Function Name: motor_io_init
******************************************************
* Summary:
*
* Parameters:
*  void
*
* Return:
*
*
*****************************************************/
static void motor_io_init(void)
{ 
	// Enable MOTOR_PWM output
	cyhal_pwm_init(
			&servo_pwm_obj, 
			PIN_SERVO_PWM, 
			NULL);
}


/*****************************************************
* Function Name: motor_init
******************************************************
* Summary:
* Initializes the IO pin used to control the motor
*
* Parameters:
*  void
*
* Return:
*
*
*****************************************************/
void motor_init(void)
{
	motor_io_init();
}


/*****************************************************
* Function Name: motor_set_pwm
******************************************************
* Summary:
*
* Parameters: angle
*  void
*
* Return:
*
*
*****************************************************/
void motor_set_pwm(int angle){

	// stop signal briefly before changing it
	rslt = cyhal_pwm_stop(&servo_pwm_obj);

	// pulse width range 500us - 2500us
	// neutral position 1500us
	
	// Calcullate pulse-width from angle
	int pulse_width = (angle*2500)/180 + 500;

	// Saturate width if out-of-bounds
	int set_width = pulse_width;
	if(pulse_width > 2500) set_width = 2500;
	if(pulse_width < 500) set_width = 500;
	//printf("Servo angle set to %d degrees (%d us)\r\n", angle, set_width);

	rslt = cyhal_pwm_stop(&servo_pwm_obj);

	rslt = cyhal_pwm_set_period(&servo_pwm_obj, 20000, set_width);

	rslt = cyhal_pwm_start(&servo_pwm_obj);
}
