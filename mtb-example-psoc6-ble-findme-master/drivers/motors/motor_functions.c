/*
 * push_button.c
 *
 *  Created on: October 9, 2023
 *      Author: Harry Kiffel
 */

#include "motor_functions.h"


cy_rslt_t   rslt;

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
	cyhal_pwm_stop(motorA.motor_pwm[1]);
	cyhal_pwm_stop(motorB.motor_pwm[0]);
	cyhal_pwm_stop(motorB.motor_pwm[1]);
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
	else if(direction == 0){
		motor->sig1active = 0;
		motor->sig2active = 0;
	}
}

void set_wheel_direction(struct MOTOR *motor, int direction){
	if(motor->name == 'b'){
		set_motor_direction(motor, -direction);
	} else{
		set_motor_direction(motor, direction);
	}
}

void set_motor_duty(struct MOTOR *motor, int duty){
	int newDuty = duty;
	if(duty > 100) {newDuty = 90;}
	else if(duty < 0) { newDuty = 0;}
	else{ newDuty = 0;}
	motor->duty = newDuty;
}

void set_motor_speed_mps(struct MOTOR *motor, float speed){
	printf("raw speed %f\r\n", speed);
	set_wheel_direction(motor, signf(speed));
	printf("Speed double: %f, Speed int: %d\r\n", (MPStoDC*fabsf(speed)), (int) (MPStoDC*fabsf(speed)));
	int duty = MPStoDC*fabsf(speed);
	int newDuty = duty;
	if(duty > 100) {newDuty = 90;}
	else if(duty < 0) { newDuty = 0;}
	else{ newDuty = 0;}
	motor->duty = (int) (newDuty);
}

void set_drive_direction(int direction){
	
	// forward
	if(direction > 0){
		set_motor_direction(&motorA, 1);
		set_motor_direction(&motorB, -1);
	}
	// reverse
	else if(direction < 0){
		set_motor_direction(&motorA, -1);
		set_motor_direction(&motorB, 1);
	}
	// brake
	else if(direction == 0){
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
	set_drive_motor_signal(&motorB, 1, motorB.sig1active * motorB.duty);
	set_drive_motor_signal(&motorB, 2, motorB.sig2active * motorB.duty);
}

void print_motor(struct MOTOR * motor){
	printf("=================================\r\n");
	printf("Motor Name: %c\r\n", motor->name);
	printf("Duty: %d, Dir: %d, sig1active: %d, sig2active: %d\r\n", motor->duty, motor->direction, motor->sig1active, motor->sig2active);
	printf("=================================\r\n");
}
