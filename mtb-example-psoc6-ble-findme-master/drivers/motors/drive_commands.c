/*
 * drive_commands.c
 *
 *  Created on: November 3, 2023
 *      Author: Harry Kiffel
 */

#include "drive_commands.h"


void * charToMotor(char c){
    if(motorA.name == tolower(c)) return &motorA;
    else if(motorB.name == tolower(c)) return &motorB;
    return 0;
}

void singleDrive(char name, int signal){

	int signal_index = signal -1;
	struct MOTOR * motor = charToMotor(name);
	cyhal_pwm_t * pwm_obj = motor->motor_pwm[signal_index];

	printf("Motor%c signal %d test \r\n", name, signal);
	
	cyhal_pwm_stop(pwm_obj);
	for(int i = 4; i < 11; i++){
		printf("Duty: %d \r\n", i*10);
		cyhal_pwm_stop(pwm_obj);
		// set duty cycle to duty
		cyhal_pwm_set_duty_cycle(pwm_obj, i*10, DRV_PWM_FREQ);
		// Start the PWM output
    	cyhal_pwm_start(pwm_obj);
		cyhal_system_delay_ms(2000);
	}
	cyhal_pwm_stop(pwm_obj);
	printf("End Motor %c signal %d test\r\n", name, signal);
}

bool isMotorString(char * str){
    char sig_str[2];
	sig_str[0] = str[0];
	sig_str[1] = str[1];
    if(	((sig_str[0] == '1') || (sig_str[0] == '2')) && 
		((sig_str[1] == 'a') || (sig_str[1] == 'b')))
    {
        return true;
    }
    return false;
}

void DriveMotor(struct MOTOR * motor, char * sig_str, int duty){
	printf("Current Motor Status\r\n");
	print_motor(motor);
	
	if(strncmp(sig_str, "cc", 2) == 0){
		set_motor_duty(motor, duty);
		set_motor_direction(motor, 1);
		drive_update();
		print_motor(motor);
		cyhal_system_delay_ms(5000);
		set_motor_duty(motor, 0);
		drive_update();
		print_motor(motor);
		
	}
	else if(strncmp(sig_str, "cw", 2) == 0){
		set_motor_duty(motor, duty);
		set_motor_direction(motor, -1);
		drive_update();
		print_motor(motor);
		cyhal_system_delay_ms(5000);
		set_motor_duty(motor, 0);
		drive_update();
		print_motor(motor);
		
	}
	else if(strncmp(sig_str, "br", 2) == 0){
		set_motor_duty(motor, 0);
		set_motor_direction(motor, 0);
		drive_update();
		print_motor(motor);
	}
}

void DriveBot(char * sig_str, int duty){
	printf("Current Motor Status\r\n");
	print_motor(&motorA);
	print_motor(&motorB);
	
	if(strncmp(sig_str, "fr", 2) == 0){
		set_drive_duty(duty);
		set_drive_direction(FORWARD);
		drive_update();
		print_motor(&motorA);
		print_motor(&motorB);
		cyhal_system_delay_ms(2000);
		set_drive_duty(0);
		drive_update();
	}
	else if(strncmp(sig_str, "bk", 2) == 0){
		set_drive_duty(duty);
		set_drive_direction(REVERSE);
		drive_update();
		print_motor(&motorA);
		print_motor(&motorB);
		cyhal_system_delay_ms(2000);
		set_drive_duty(0);
		drive_update();
	}
	else if(strncmp(sig_str, "br", 2) == 0){
		set_drive_direction(0);
		set_drive_duty(0);
		drive_update();
	}
	print_motor(&motorA);
	print_motor(&motorB);
}



// void DriveMotorA(char * sig_str, int duty){
//     printf("Current Motor Status\r\n");
// 	print_motor(&motorA);
	
// 	if(strncmp(sig_str, "cc", 2) == 0){
// 		set_drive_motor_speed(&motorA, duty);
// 		set_drive_motor_direction(&motorA, 1);
// 		print_motor(&motorA);
// 		cyhal_system_delay_ms(5000);
// 		set_drive_motor_speed(&motorA, 0);
// 		print_motor(&motorA);
		
// 	}
// 	else if(strncmp(sig_str, "cw", 2) == 0){
// 		set_drive_motor_speed(&motorA, duty);
// 		set_drive_motor_direction(&motorA, -1);
// 		print_motor(&motorA);
// 		cyhal_system_delay_ms(5000);
// 		set_drive_motor_speed(&motorA, 0);
// 		print_motor(&motorA);
		
// 	}
// 	else if(strncmp(sig_str, "br", 2) == 0){
// 		set_drive_motor_speed(&motorA, 0);
// 		set_drive_motor_direction(&motorA, 0);
// 		set_drive_motor_signal(&motorA, 1, 0);
// 		set_drive_motor_signal(&motorA, 2, 0);
// 		print_motor(&motorA);
// 	}
// }
