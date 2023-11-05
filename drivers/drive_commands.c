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