
/******************************************************************************
* File Name: main.c
*
* Description: This is source code for the PSoC 6 MCU with BLE Find Me code
*              example.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/******************************************************************************
* Header files
******************************************************************************/

#include "cy_pdl.h"
#include "drivers/console.h"
#include "cybsp.h"
#include "cyhal.h"
#include "ble_findme.h"
#include "main.h"

// Enables --------------------

// Debug console mode
#define DEBUG_CONSOLE_MODE 1
#define CMD_LENGTH (100u)

// Device enables
// SPI device enables
#define ENABLE_SPI 1
// I2C device enables
#define ENABLE_I2C 1
// PWM device enables
#define ENABLE_MOTOR 1
#define ENABLE_ULTRASONIC 1

// Bot state information
botstate state[NUM_BOTS];

void swarm_main() {

	// Gather new sensor data into own botstate
	botstate my_state = state[0];

	// IR
	ir_read_all_until_valid(5, false);
	for (int i = 0; i < NUM_IR; i++) {
		my_state.ir_data[i] = multi_ir_data_store[i];
	}

	// Aim ultrasonic at nearest ir

	// US
	my_state.us_echo1_cm = ultrasonic_get_object_distance(PIN_ECHO1);
	my_state.us_echo2_cm = ultrasonic_get_object_distance(PIN_ECHO2);
	my_state.servo_ang_rad = ultrasonic_angle;

	// IMU

	// Save, report
	state[0] = my_state;
	print_botstate(state);

	// Collect other botstates over bt

	// Pass botstates to swarm algorithm to get next position

	// Move to position
}

int main(void) {

    console_init();

    printf("\x1b[2J\x1b[;H");

    printf("******************\n\r");
    printf("* ECE453 Dev Platform\n\r");
    printf("******************\n\r");
    printf("* -- Initializing user push button\n\r");
	push_button_init();

    printf("* -- Initializing user LED\n\r");
    leds_init();

    printf("* -- Enabling Interrupts\n\r");
    /* Enable global interrupts */
    	__enable_irq();

	// I2C-based devices init
	if (ENABLE_I2C) {
		printf("* -- Initializing I2C Bus\n\r");
		i2c_init();
		//ir_boot();
	}

	#if ENABLE_SPI
		printf("* -- Initializing SPI Bus\n\r");
		if (spi_init() == CY_RSLT_SUCCESS) {
			if (imu_cs_init() != CY_RSLT_SUCCESS) {
				printf("Error: Failed to initialize IMU chip select\n\r");
			}
		} else {
			printf("Error: Failed to initialize SPI\n\r");
		}
	#endif

	#if ENABLE_MOTOR
		printf("* -- Initializing Motor Functions\n\r");
		drive_motor_init();
	#endif

	#if ENABLE_ULTRASONIC
		printf("* -- Initializing Ultrasonic Functions\n\r");
		ultrasonic_init();
	#endif

    printf("* -- Initializing BT Functions\n\r");
    ble_findme_init();

	printf("Initialization complete. \r\n");
	printf("****************** \r\n\n");

    while(1) {
        ble_findme_process();

		if (DEBUG_CONSOLE_MODE == 1) {
			// Command conditions
			char cmdStr[CMD_LENGTH];
			if (ALERT_CONSOLE_RX || ALERT_BT_RX) {
				if (ALERT_BT_RX) {
					strcpy(cmdStr, btInputString);
				} else {
					strcpy(cmdStr, pcInputString);
				}

				// IR test commands
				if (strncmp(cmdStr, "IR test", 7) == 0) {
					char ch_num_str = cmdStr[8];
					int ch_num = atoi(&ch_num_str);
					ir_io_test(ch_num);
				} else if (strncmp(cmdStr, "IR read all", 11) == 0) {
					ir_read_all(5, true);
				} else if (strncmp(cmdStr, "IR read valid", 13) == 0) {
					int max = 1000;
					char ch_num_str = cmdStr[14];
					int ch_num = atoi(&ch_num_str);
					ir_read_until_valid(ch_num, max, true);
				} else if (strncmp(cmdStr, "IR read", 7) == 0) {
					char ch_num_str = cmdStr[8];
					int ch_num = atoi(&ch_num_str);
					ir_read(ch_num, 5, true);
					int us_angle = 180-(ch_num*90);	// rotates channel assignment for easy angle math
					printf("Rotating to %d degrees \r\n", us_angle);
					set_servo_angle(us_angle);
				} else if (strncmp(cmdStr, "IR reboot", 9) == 0) {
					ir_boot();
				
				// Servo test commands
				}  else if (strncmp(cmdStr, "servo ", 6) == 0) {
					int servo_angle = atoi(&cmdStr[6]);
					printf("CMD result: Setting servo angle to %d\r\n", servo_angle);
					set_servo_angle(servo_angle);
				}  else if (strncmp(cmdStr, "servo_us ", 9) == 0) {
					int width = atoi(&cmdStr[9]);
					printf("CMD result: Setting servo width to %d\r\n", width);
					set_servo_us(width);
					if(width < 500 || width > 2500){
						printf("CMD warning: Servo angle was out-of-bounds and rectified to 0 or 180\r\n");
					}

				// US test commands
				} else if (strncmp(cmdStr, "distance ", 9) == 0) {
			  		//int servo_angle = atoi(&cmdStr[9]);
			  		printf("CMD result: Read Distance measurements.\r\n");
			  		// uint32_t echodist1;
			  		// uint32_t echodist2;
			  		printf("ECHO 1: %f cm\r\n", ultrasonic_get_object_distance(PIN_ECHO1));
			  		printf("ECHO 2: %f cm\r\n", ultrasonic_get_object_distance(PIN_ECHO2));
			  	} else if (strncmp(cmdStr, "locate", 6) == 0) {
			  		printf("CMD result: Locating Object (if present).\r\n");
			  		ultrasonic_locate_object();
			  	
				// Motors test commands
				}
				else if (strncmp(cmdStr, "motors ", 7) == 0){
			  		char sig_str[2];
			  		sig_str[0] = cmdStr[7];
			  		sig_str[1] = cmdStr[8];
			  		int duty = atoi(&cmdStr[10]);
			  		DriveMotor(&motorA, sig_str, duty);
					cyhal_system_delay_ms(1000);
			  		DriveMotor(&motorB, sig_str, duty);
			  	} else if (strncmp(cmdStr, "drive ", 6) == 0){
			  		char sig_str[2];
			  		sig_str[0] = cmdStr[6];
			  		sig_str[1] = cmdStr[7];
			  		int duty = atoi(&cmdStr[9]);
					DriveBot(sig_str, duty);
				} else if (strncmp(cmdStr, "test_motor ", 11) == 0){
					char sig_str[2];
			  		sig_str[0] = cmdStr[11];
			  		int signal = atoi(cmdStr[12]);
					singleDrive(sig_str[0], signal); 

				// IMU test commands
				} else if (strncmp(cmdStr, "IMU read", 8) == 0) {
					if (ENABLE_SPI) {
						imu_orientation();
					} else {
						printf("CMD fail: SPI not enabled.\r\n");
					}

				// BT chain commands
				} else if (strncmp(cmdStr, "BT chain start", 14) == 0) {
					printf("CMD result: Starting BT chain\r\n");
					ble_chain_start();
					swarm_main();
				} else if (strncmp(cmdStr, "BT chain join", 13) == 0) {
					printf("CMD result: Joining BT chain\r\n");
					ble_chain_join();
					swarm_main();
				} else if(strncmp(cmdStr, "navmode", 7) == 0){
					int index = 0;
    				int waypoint_index;
    				int updateDelay = 100;  // 100 ms 10Hz
    				while(!waypoint_complete){
    				    waypoint_index = (index % QLENGTH);
    				    // target_pose = newPose();
    				    // IMU_read();
    				    createWaypointPath(estimated_pose);
    				    path_follow(estimated_pose);
    				    if(waypoint_complete){
    				        index++;
    				    }

    				    cyhal_system_delay_ms(updateDelay);
						cInputIndex = 0;
						ALERT_CONSOLE_RX = false;
						ALERT_BT_RX = false;
						if(strncmp(cmdStr, "waypoint ", 9) == 0){
							waypoint_complete = false;
							struct POSE newPose;
							char xStr[2] = {cmdStr[9],  cmdStr[10]};
							char yStr[2] = {cmdStr[12], cmdStr[13]};
							double x = atof(&xStr);
							double y = atof(&yStr);
							newPose.x = 	x;
							newPose.y = 	y;
							newPose.theta =	0;
							waypoints[waypoint_index+1] = newPose;
						}
    				}

				// Command fail
				} else {
					printf("CMD fail: command not recognized.\r\n");
				}

				cInputIndex = 0;
				ALERT_CONSOLE_RX = false;
				ALERT_BT_RX = false;
			}
		}
	}
};

/* END OF FILE */