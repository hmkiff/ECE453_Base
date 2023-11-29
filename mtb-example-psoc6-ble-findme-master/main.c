
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

#define SWARM_MODE 0

// Simbot enables
// Should this PSoC6 board fake sensor data?
#define SIMBOT_HOST 0
// If SIMBOT_HOST is 1, how many bots should be faked?
#define NUM_SIMBOTS 3

// Device enables
// SPI device enables
#define ENABLE_SPI 0
#define ENABLE_IMU 0
#define ENABLE_EEPROM 0
// I2C device enables
#define ENABLE_I2C 1
#define ENABLE_TEMP 0
#define ENABLE_IR_MUX 1
#define ENABLE_IR 1
#define ENABLE_IO_EXPANDER 0
// PWM device enables
#define ENABLE_MOTOR 0
#define ENABLE_ULTRASONIC 0

// Bot state information
botstate state[NUM_BOTS];

// IR object
VL53LX_Dev_t IR_dev_2;

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
		if (ENABLE_IR) {
			for (int i = 0; i < 4; i++) {
				cy_rslt_t result = ir_mux_set_chnl(i);
				if (result != CY_RSLT_SUCCESS) {
					printf("IR Error: Unable to set IR channel to %i, reason:\n\r", i);
					print_result(result);
				}
				bool ir_wait_boot = true;
				if (ir_wait_boot) {
					printf("* -- Waiting for IR %i boot\n\r", i);
					VL53LX_Error err = VL53LX_WaitDeviceBooted(&IR_dev_2);
					if (err != VL53LX_ERROR_NONE) {
						printf("IR Error: Unable to detect IR %i boot, reason:\r\n", i);
						print_IR_error(err);
					}
				}
				printf("* -- Initializing IR data\n\r");
				VL53LX_Error err = VL53LX_DataInit(&IR_dev_2);
				if (err != VL53LX_ERROR_NONE) {
					printf("CMD fail: Unable to initialize IR %i data, reason:\r\n", i);
					print_IR_error(err);
				}
			}
		}
	}

	if (ENABLE_SPI) {
		printf("* -- Initializing SPI Bus\n\r");
		cy_rslt_t rslt = spi_init();
		if (rslt == CY_RSLT_SUCCESS) {
			if (ENABLE_EEPROM) {
				if (eeprom_cs_init() == CY_RSLT_SUCCESS) {
					if(eeprom_test() != CY_RSLT_SUCCESS) {
						// Something is wrong wit the EEPROM
						while(1){};
					}
				}
			}
			if (ENABLE_IMU) {
				cy_rslt_t rslt = imu_cs_init();
				if (rslt != CY_RSLT_SUCCESS) {
					print_result(rslt);
					return -1;
				}
			}
		} else {
			print_result(rslt);
			return -1;
		}
	}

	#if ENABLE_MOTOR
		printf("* -- Initializing Motor Functions\n\r");
		drive_motor_init();
	#endif

	#if ENABLE_ULTRASONIC
		printf("* -- Initializing Ultrasonic Functions\n\r");
		ultrasonic_init();
		motor_init();
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
				if (strncmp(cmdStr, "IR sel ", 7) == 0) {
					#if ENABLE_I2C
						#if ENABLE_IR_MUX
							char ch_num_str = cmdStr[7];
							int ch_num = atoi(&ch_num_str);
							cy_rslt_t result = ir_mux_set_chnl(ch_num);
							if (result != CY_RSLT_SUCCESS) {
								print_result(result);
								printf("CMD fail: Couldn't switch IR, see error above\r\n");
							} else {
								printf("CMD result: IR swicthed to ch. %i\r\n", ch_num);
							}
						#else
							printf("CMD fail: io expander not enabled.\r\n");
						#endif
					#else
						printf("CMD fail: I2C not enabled.\r\n");
					#endif
				} else if (strncmp(cmdStr, "IR test", 7) == 0) {
					uint8_t byteData;
					uint16_t wordData;
					VL53LX_RdByte(&IR_dev_2, 0x010F, &byteData);
					printf("VL53LX Model_ID: %02X\n\r", byteData);
					VL53LX_RdByte(&IR_dev_2, 0x0110, &byteData);
					printf("VL53LX Module_Type: %02X\n\r", byteData);
					VL53LX_RdWord(&IR_dev_2, 0x010F, &wordData);
					printf("VL53LX: %02X\n\r", wordData);
					i2c_test(&IR_dev_2);
				} else if (strncmp(cmdStr, "IR read", 7) == 0) {
					#if ENABLE_I2C
						#if ENABLE_IR
							int num_measurements = 5;
							printf("CMD result: Starting IR measurement 1 of %i...\r\n", num_measurements);
							VL53LX_Error err = VL53LX_StartMeasurement(&IR_dev_2);
							if (err != VL53LX_ERROR_NONE) {
								printf("CMD fail: IR encountered an error starting a measurement\r\n");
								print_IR_error(err);
							} else {
								for (int i = 2; i < num_measurements; i++) {
									printf("CMD result: IR measurement started. Waiting until ready...\r\n");
									err = VL53LX_WaitMeasurementDataReady(&IR_dev_2);
									if (err != VL53LX_ERROR_NONE) {
										printf("CMD fail: IR encountered an error waiting for a measurement\r\n");
										print_IR_error(err);
									} else {
										VL53LX_MultiRangingData_t data;
										err = VL53LX_GetMultiRangingData(&IR_dev_2, &data);
										if (err != VL53LX_ERROR_NONE) {
											printf("CMD fail: IR encountered an error while getting measurement data\r\n");
											print_IR_error(err);
										} else {
											printf("CMD result: IR sees furthest object %i mm away\r\n", data.RangeData->RangeMaxMilliMeter);
											printf("CMD result: IR sees closest object %i mm away\r\n", data.RangeData->RangeMinMilliMeter);
											printf("CMD result: IR sees %i objects\r\n", data.NumberOfObjectsFound);
										}
									}
									printf("CMD result: Starting IR measurement %i of %i...\r\n", i, num_measurements);
									VL53LX_Error err = VL53LX_ClearInterruptAndStartMeasurement(&IR_dev_2);
									if (err != VL53LX_ERROR_NONE) {
										printf("CMD fail: IR encountered an error starting a measurement\r\n");
										print_IR_error(err);
									}
								}
							}
						#else
							printf("CMD fail: IR not enabled.\r\n");
						#endif
					#else
						printf("CMD fail: I2C not enabled.\r\n");
					#endif
				} else if (strncmp(cmdStr, "step dir ", 9) == 0) {
					char step_dir = cmdStr[9];
					bool motor_dir = 0;
					bool valid_flag = 1;
					if(step_dir == 'r') {motor_dir = 1 ;}
					else if(step_dir == 'l') {motor_dir = 0;}
					else {valid_flag = 0;}
					if (valid_flag) {
						printf("CMD result: Setting stepper direction to %s\r\n", motor_dir ? "right" : "left");
						motor_set_dir(motor_dir);
					} else {
						printf("CMD fail: Invalid direction command. Check case!\r\n");
					}
				} else if (strncmp(cmdStr, "step speed ", 11) == 0) {
					int step_speed = atoi(&cmdStr[11]);
					printf("CMD result: Setting stepper speed to %d\r\n", step_speed);
					motor_step_speed(step_speed);
					if(step_speed < 0 || step_speed > 200){
						printf("CMD warning: Stepper speed was out-of-bounds and rectified to 0 or 200\r\n");
					}
				} else if (strncmp(cmdStr, "servo ", 6) == 0) {
					int servo_angle = atoi(&cmdStr[6]);
					printf("CMD result: Setting servo angle to %d\r\n", servo_angle);
					motor_set_pwm(servo_angle);
					if(servo_angle < 0 || servo_angle > 180){
						printf("CMD warning: Servo angle was out-of-bounds and rectified to 0 or 180\r\n");
					}
				} else if (strncmp(cmdStr, "distance ", 9) == 0) {
					//int servo_angle = atoi(&cmdStr[9]);
					printf("CMD result: Read Distance measurements.\r\n");
					// uint32_t echodist1;
					// uint32_t echodist2;
					printf("ECHO 1: %f cm\r\n", ultrasonic_get_object_distance(PIN_ECHO1));
					printf("ECHO 2: %f cm\r\n", ultrasonic_get_object_distance(PIN_ECHO2));
				} else if (strncmp(cmdStr, "single_drive ", 13) == 0) {
					char sig_str[2];
					sig_str[0] = cmdStr[13];
					sig_str[1] = cmdStr[14];
					int duty = atoi(&cmdStr[16]);
					if(isMotorString(sig_str)) {
						singleDrive(charToMotor(cmdStr[14]), atoi(&cmdStr[13]), duty);
					} else {
						printf("No motor signal specified. enter signal name after CMD. 1a, 2a, 1b, 2b");
					}
				} else if (strncmp(cmdStr, "motors ", 7) == 0){
					char sig_str[2];
					sig_str[0] = cmdStr[7];
					sig_str[1] = cmdStr[8];
					int duty = atoi(&cmdStr[10]);
					DriveMotor(&motorA, sig_str, duty);
					DriveMotor(&motorB, sig_str, duty);
				} else if (strncmp(cmdStr, "IMU ", 4) == 0) {
					if (ENABLE_SPI) {
						if (ENABLE_IMU) {
							// IMU read here
						} else {
							printf("CMD fail: IMU not enabled.\r\n");
						}
					} else {
						printf("CMD fail: SPI not enabled.\r\n");
					}
				} else {
					printf("CMD fail: command not recognized.\r\n");
				}
				cInputIndex = 0;
				ALERT_CONSOLE_RX = false;
				ALERT_BT_RX = false;
			}
		} else if (SIMBOT_HOST == 1) {
			
		} else if (SWARM_MODE == 1) {
			// Swarm mode
			// Sweep ultrasonic

			// Gather new sensor data into own botstate
			state[0].us_echo1_cm = ultrasonic_get_object_distance(PIN_ECHO1);
			state[0].us_echo2_cm = ultrasonic_get_object_distance(PIN_ECHO2);
			state[0].servo_ang_rad = 0;

			// Collect other botstates over bt

			// Pass botstates to swarm algorithm to get next position
			botpos next_pos = swarm(state, 0);

			// Move to position
		}
    }
}


/* END OF FILE */