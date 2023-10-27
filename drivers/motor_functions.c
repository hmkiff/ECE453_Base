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



	// stop signal briefly before changing it
	rslt = cyhal_pwm_stop(&drive1A_pwm_obj);
	// Start the PWM output
    rslt = cyhal_pwm_start(&drive1A_pwm_obj);

// Drive motor functions

void drive_motor_init(void)
{
	// init drive motor A pwms
	cyhal_pwm_init( &drive1A_pwm_obj, PIN_MOTOR_1A, NULL);
	cyhal_pwm_init( &drive2A_pwm_obj, PIN_MOTOR_2A, NULL);
	// init drive motor B pwms
	cyhal_pwm_init( &drive1B_pwm_obj, PIN_MOTOR_1B, NULL);
	cyhal_pwm_init( &drive2B_pwm_obj, PIN_MOTOR_2B, NULL);

	// Set a duty cycle of 0% (coast) and frequency of 20kHz
	// motorA
    cyhal_pwm_set_duty_cycle(&drive1A_pwm_obj, 0, DRV_PWM_FREQ);
	cyhal_pwm_set_duty_cycle(&drive2A_pwm_obj, 0, DRV_PWM_FREQ);
	// motorB
	cyhal_pwm_set_duty_cycle(&drive1B_pwm_obj, 0, DRV_PWM_FREQ);
	cyhal_pwm_set_duty_cycle(&drive2B_pwm_obj, 0, DRV_PWM_FREQ);

}

void set_drive_motor_signal(MOTOR *motor, int signal, int duty)
{
	// obtain reference to motor's pwm signal.
	int signal_index = signal - 1;
	cyhal_pwm_t * drive_pwm_obj = motor->motor_pwm[signal_index];

	// stop signal briefly before changing it
	cyhal_pwm_stop(&drive_pwm_obj);

	// set duty cycle to duty
	cyhal_pwm_set_duty_cycle(&drive_pwm_obj, duty, DRV_PWM_FREQ);
	
	// Start the PWM output
    cyhal_pwm_start(&drive_pwm_obj);
}

void set_drive_motor_direction(MOTOR *motor, int direction)
{
	// direction -1 reverse, 0 brake, 1 forward
	if(direction < 0){
		motor->direction = -1;
		set_drive_motor_signal(&motor, motor->motor_pwm[0], motor->duty);
		set_drive_motor_signal(&motor, motor->motor_pwm[1], 100);
	}
	else if(direction == 0){
		motor->direction = 0;
		set_drive_motor_signal(&motor, motor->motor_pwm[0], 100);
		set_drive_motor_signal(&motor, motor->motor_pwm[1], 100);
	}
	else if(direction > 0){
		motor->direction = 1;
		set_drive_motor_signal(&motor, motor->motor_pwm[0], 100);
		set_drive_motor_signal(&motor, motor->motor_pwm[1], motor->duty);
	}
}

void set_drive_motor_speed(MOTOR *motor, int duty)
{
	direction = motor->direction;
	if(direction < 0){
		set_drive_motor_signal(&motor, motor->motor_pwm[0], duty);
		set_drive_motor_signal(&motor, motor->motor_pwm[1], 100);
	}
	else if(direction > 0){
		set_drive_motor_signal(&motor, motor->motor_pwm[0], 100);
		set_drive_motor_signal(&motor, motor->motor_pwm[1], duty);
	}
	else{
		set_drive_motor_signal(&motor, motor->motor_pwm[0], 100);
		set_drive_motor_signal(&motor, motor->motor_pwm[1], 100);
	}
}

void set_drive_motor_speed_rpm(MOTOR *motor, int speed_rpm)
{
	int duty_percent;
	if(speed_rpm > 200) duty_percent = 100;
	else if(speed_rpm < 0) duty_percent = 0;
	set_drive_motor_speed(&motor, duty_percent);

}

void set_drive_move_direction(int move_dir)
{
}

void set_drive_turn_direction(int turn_dir)
{
}

void set_drive_speed(int duty)
{
}

void set_drive_speed_rpm(int speed_rpm)
{
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
    /* ADD CODE to configure motor pins as outputs */
	// Enable MOTOR_DIR output
	cyhal_gpio_init(
			PIN_MOTOR_DIR,            // Pin
			CYHAL_GPIO_DIR_OUTPUT,      // Direction
			CYHAL_GPIO_DRIVE_STRONG,     // Drive Mode
			true);
	// Enable MOTOR_STEP output
	// Enable MOTOR_PWM output
	cyhal_pwm_init(
			&step_pwm_obj, 
			PIN_MOTOR_STEP, 
			NULL);
	// Enable MOTOR_PWM output
	cyhal_pwm_init(
			&servo_pwm_obj, 
			PIN_MOTOR_PWM, 
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
* Function Name: motor_dir_inc
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
void motor_set_dir(bool direction){
	if(DIR == direction){
		//printf("MOTOR_MSG: Stepper direction is already %s\r\n", direction ? "right": "left");
		cyhal_gpio_write(PIN_MOTOR_DIR, direction);
	}
	else{
		//printf("MOTOR_MSG: Stepper direction set %s\r\n", direction ? "right": "left");
		cyhal_gpio_write(PIN_MOTOR_DIR, direction);
		DIR = direction;
	}
}

/*****************************************************
* Function Name: motor_step_speed
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
void motor_step_speed(int speed){
	
	// stop signal briefly before changing it
	rslt = cyhal_pwm_stop(&step_pwm_obj);

	// Saturate speed if out-of-bounds
	int set_speed = speed;
	if(speed > 200) set_speed = 200;
	if(speed < 0) set_speed = 0;

	// Set a duty cycle of 50% and frequency of set_speed
    rslt = cyhal_pwm_set_duty_cycle(&step_pwm_obj, 50, set_speed);
 
    // Start the PWM output
    rslt = cyhal_pwm_start(&step_pwm_obj);
}

/*****************************************************
* Function Name: motor_set_pwm
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
