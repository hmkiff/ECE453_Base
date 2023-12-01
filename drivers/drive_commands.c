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

void singleDrive(struct MOTOR * motor, int signal, int duty){

    int duration = 10;
	printf("Running motor signal %d%c at %d%% for %d seconds.\r\n", signal, motor->name, duty, duration);
    set_drive_motor_signal(motor, signal, duty);
    print_motor(motor);
	for(int i = duration; i > 0; i--){
        cyhal_system_delay_ms(1000);
	    printf(".\r\n");
    }
	printf("Ending motor signal %d%c.\r\n", signal, motor->name);
	set_drive_motor_signal(motor, signal, 0);
}

void DriveMotorA(char * sig_str, int duty){
    printf("Current Motor Status\r\n");
	print_motor(&motorA);
	
	if(strncmp(sig_str, "cc", 2) == 0){
		set_drive_motor_speed(&motorA, duty);
		set_drive_motor_direction(&motorA, 1);
		print_motor(&motorA);
		cyhal_system_delay_ms(5000);
		set_drive_motor_speed(&motorA, 0);
		print_motor(&motorA);
		
	}
	else if(strncmp(sig_str, "cw", 2) == 0){
		set_drive_motor_speed(&motorA, duty);
		set_drive_motor_direction(&motorA, -1);
		print_motor(&motorA);
		cyhal_system_delay_ms(5000);
		set_drive_motor_speed(&motorA, 0);
		print_motor(&motorA);
		
	}
	else if(strncmp(sig_str, "br", 2) == 0){
		set_drive_motor_speed(&motorA, 0);
		set_drive_motor_direction(&motorA, 0);
		set_drive_motor_signal(&motorA, 1, 0);
		set_drive_motor_signal(&motorA, 2, 0);
		print_motor(&motorA);
	}
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
		set_drive_motor_speed(motor, duty);
		set_drive_motor_direction(motor, 1);
		print_motor(motor);
		cyhal_system_delay_ms(5000);
		set_drive_motor_speed(motor, 0);
		print_motor(motor);
		
	}
	else if(strncmp(sig_str, "cw", 2) == 0){
		set_drive_motor_speed(motor, duty);
		set_drive_motor_direction(motor, -1);
		print_motor(motor);
		cyhal_system_delay_ms(5000);
		set_drive_motor_speed(motor, 0);
		print_motor(motor);
		
	}
	else if(strncmp(sig_str, "br", 2) == 0){
		set_drive_motor_speed(motor, 0);
		set_drive_motor_direction(motor, 0);
		set_drive_motor_signal(motor, 1, 0);
		set_drive_motor_signal(motor, 2, 0);
		print_motor(motor);
	}
}

void DriveBot(char * sig_str, int duty){
	printf("Current Motor Status\r\n");
	print_motor(&motorA);
	print_motor(&motorB);
	
	if(strncmp(sig_str, "fr", 2) == 0){
		set_drive_speed(duty);
		set_drive_move_direction(FORWARD);
		print_motor(&motorA);
		print_motor(&motorB);
		cyhal_system_delay_ms(2000);
		set_drive_speed(0);
	}
	else if(strncmp(sig_str, "bk", 2) == 0){
		set_drive_speed(duty);
		set_drive_move_direction(REVERSE);
		print_motor(&motorA);
		print_motor(&motorB);
		cyhal_system_delay_ms(2000);
		set_drive_speed(0);
	}
	else if(strncmp(sig_str, "br", 2) == 0){
		set_drive_move_direction(0);
		set_drive_speed(0);
	}
	print_motor(&motorA);
	print_motor(&motorB);
}
