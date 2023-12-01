/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty PSoC6 Application
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2019-2021, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

#include <stdlib.h>
#include <math.h>

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "main.h"
#include "swarm/botstate.h"
#include "swarm/swarm.h"

// Enables --------------------

// Debug console mode
#define DEBUG_CONSOLE_MODE 1

// Simbot enables
// Should this PSoC6 board fake sensor data?
#define SIMBOT_HOST 0
// If SIMBOT_HOST is 1, how many bots should be faked?
#define NUM_SIMBOTS 3

// Device enables
// SPI device enables
#define ENABLE_SPI 0
// I2C device enables
#define ENABLE_I2C 0
#define ENABLE_TEMP 0
#define ENABLE_IR_MUX 0
#define ENABLE_IR 0
#define ENABLE_IO_EXPANDER 0
// PWM device enables
#define ENABLE_MOTOR 1
#define ENABLE_ULTRASONIC 1

// Bot state information
botstate state[NUM_BOTS];

int main(void) {
	float temp;
	uint8_t led_mask = 0x00;

    console_init();

    printf("\x1b[2J\x1b[;H");

    printf("******************\n\r");
    printf("* ECE453 Dev Platform\n\r");

    printf("* -- Initializing user push button\n\r");
	push_button_init();

    printf("* -- Initializing user LED\n\r");
    leds_init();

    printf("* -- Enabling Interrupts\n\r");
    /* Enable global interrupts */
    	__enable_irq();

// I2C-based devices init
#if ENABLE_I2C
    printf("* -- Initializing I2C Bus\n\r");
    i2c_init();
	#if ENABLE_TEMP
		float temp;
	#endif
	#if ENABLE_IO_EXPANDER
		printf("* -- Initializing I/O Expander\n\r");
		uint8_t led_mask = 0x00;
		io_expander_set_all_out();
	#endif
	#if ENABLE_IR_MUX
		printf("* -- Initializing IR Mux\n\r");
		cy_rslt_t result = cyhal_gpio_init(IR_MUX_RST_PIN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
		if (result != CY_RSLT_SUCCESS) {
			print_result(result);
			printf("IR Error: Unable to initialize IR Mux, see above\n\r");
			return -1;
		}
		result = ir_mux_set_chnl(2);
		if (result != CY_RSLT_SUCCESS) {
			print_result(result);
			printf("IR Error: Unable to initialize IR Mux, see above\n\r");
			return -1;
		}
	#endif
#endif

#if ENABLE_SPI
    printf("* -- Initializing SPI Bus\n\r");
    if (spi_init() == CY_RSLT_SUCCESS)
    {
		if(eeprom_cs_init() == CY_RSLT_SUCCESS)
		{
			if(eeprom_test() != CY_RSLT_SUCCESS)
			{
				// Something is wrong wit the EEPROM
				while(1){};
			}

		}
    }
#endif

#if ENABLE_MOTOR
    printf("* -- Initializing Motor Functions\n\r");
    drive_motor_init();
#endif

#if ENABLE_ULTRASONIC
    printf("* -- Initializing Ultrasonic Functions\n\r");
    ultrasonic_init();
    motor_init();
#endif

printf("****************** \r\n\n");

    while(1)
    {
    	Cy_SysLib_Delay(1000);

		if (DEBUG_CONSOLE_MODE == 1) {
			// Command conditions
			if (ALERT_CONSOLE_RX) {
				if (strncmp(pcInputString, "IR sel ", 7) == 0) {
					#if ENABLE_I2C
						#if ENABLE_IR_MUX
							char ch_num_str = pcInputString[7];
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
				} else if (strncmp(pcInputString, "IR read", 7) == 0) {
					#if ENABLE_I2C
						#if ENABLE_IR
							while (true) {
								printf("CMD result: init test begin\r\n");
								VL53LX_Dev_t IR_dev_2;
								VL53LX_DataInit(&IR_dev_2);
							}
						#else
							printf("CMD fail: IR not enabled.\r\n");
						#endif
					#else
						printf("CMD fail: I2C not enabled.\r\n");
					#endif
			  	} else if (strncmp(pcInputString, "servo ", 6) == 0) {
				    int servo_angle = atoi(&pcInputString[6]);
				  	printf("CMD result: Setting servo angle to %d\r\n", servo_angle);
				  	motor_set_pwm(servo_angle);
				  	if(servo_angle < 0 || servo_angle > 180){
					 	printf("CMD warning: Servo angle was out-of-bounds and rectified to 0 or 180\r\n");
				  	}
			  	} else if (strncmp(pcInputString, "distance ", 9) == 0) {
			  		//int servo_angle = atoi(&pcInputString[9]);
			  		printf("CMD result: Read Distance measurements.\r\n");
			  		// uint32_t echodist1;
			  		// uint32_t echodist2;
			  		printf("ECHO 1: %f cm\r\n", ultrasonic_get_object_distance(PIN_ECHO1));
			  		printf("ECHO 2: %f cm\r\n", ultrasonic_get_object_distance(PIN_ECHO2));
			  	} else if (strncmp(pcInputString, "motors ", 7) == 0){
			  		char sig_str[2];
			  		sig_str[0] = pcInputString[7];
			  		sig_str[1] = pcInputString[8];
			  		int duty = atoi(&pcInputString[10]);
			  		DriveMotor(&motorA, sig_str, duty);
					cyhal_system_delay_ms(1000);
			  		DriveMotor(&motorB, sig_str, duty);
			  	} else if (strncmp(pcInputString, "drive ", 6) == 0){
			  		char sig_str[2];
			  		sig_str[0] = pcInputString[6];
			  		sig_str[1] = pcInputString[7];
			  		int duty = atoi(&pcInputString[9]);
					DriveBot(sig_str, duty);
				} else {
			  		printf("CMD fail: command not recognized.\r\n");
			  	}
				cInputIndex = 0;
				ALERT_CONSOLE_RX = false;
			}
		} else if (SIMBOT_HOST == 1) {
			
		} else {
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

/* [] END OF FILE */