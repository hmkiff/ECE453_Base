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

// Enables
#define ENABLE_I2C 0
#define ENABLE_SPI 0
#define ENABLE_TEMP 0
#define ENABLE_IO_EXPANDER 0
#define ENABLE_MOTOR 1
#define ENABLE_ULTRASONIC 1

int main(void)
{
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

#if ENABLE_I2C
    printf("* -- Initializing I2C Bus\n\r");
    i2c_init();
	#if ENABLE_IO_EXPANDER
		printf("* -- Initializing I/O Expander\n\r");
		io_expander_write_reg(0x03, 0x00); 	// Set all pins as outputs
		io_expander_write_reg(0x01, 0xFF); 	// Turn on all LEDs
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
#endif

printf("****************** \r\n\n");

    while(1)
    {
    	Cy_SysLib_Delay(1000);

		// Command conditions
		if (ALERT_CONSOLE_RX) {
			if (strncmp(pcInputString, "temp", 4) == 0) {
				#if ENABLE_TEMP
					temp = LM75_get_temp();
					printf("CMD Result: Temperature = %.2f\r\n", temp);
				#else
					printf("CMD fail: Temperature not enabled.\r\n");
				#endif
			} else if (strncmp(pcInputString, "led on ", 7) == 0) {
				#if ENABLE_IO_EXPANDER
					char led_num_str = pcInputString[7];
					int led_num = atoi(&led_num_str);
					if ((led_num >= 0) && (led_num <= 7)) {
						printf("CMD result: Turning LED %i on\r\n", led_num);
						led_mask = led_mask | (uint8_t)pow(2, led_num);
						io_expander_write_reg(0x01, led_mask);
					} else {
						printf("CMD fail: No LED at %i\r\n", led_num);
					}
				#else
					printf("CMD fail: io expander not enabled.\r\n");
				#endif
			} else if (strncmp(pcInputString, "led off ", 8) == 0) {
				#if ENABLE_IO_EXPANDER
					char led_num_str = pcInputString[8];
					int led_num = atoi(&led_num_str);
					if ((led_num >= 0) && (led_num <= 7)) {
						printf("CMD result: Turning LED %i off\r\n", led_num);
						led_mask = led_mask & ~(uint8_t)pow(2, led_num);
						io_expander_write_reg(0x01, led_mask);
					} else {
						printf("CMD fail: No LED at %i\r\n", led_num);
					}
				#else
					printf("CMD fail: io expander not enabled.\r\n");
				#endif
			} else if (strncmp(pcInputString, "step dir ", 9) == 0) {
				char step_dir = pcInputString[9];
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
			} else if (strncmp(pcInputString, "step speed ", 11) == 0) {
				int step_speed = atoi(&pcInputString[11]);
				printf("CMD result: Setting stepper speed to %d\r\n", step_speed);
				motor_step_speed(step_speed);
				if(step_speed < 0 || step_speed > 200){
					printf("CMD warning: Stepper speed was out-of-bounds and rectified to 0 or 200\r\n");
				}
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
			} else if (strncmp(pcInputString, "drive_motors ", 13) == 0) {
				char sig_str[2];
				sig_str[0] = pcInputString[13];
				sig_str[1] = pcInputString[14];
				//int duty = atoi(&pcInputString[16]);
				if(strncmp(sig_str, "1a", 2) == 0){
					printf("CMD result: Perform motor diagnostic .\r\n");
					printf("Sending motor signal 1A, 20Hz for 30 seconds.\r\n");
					printf("Duty Cycle: 5\r\n");
					set_drive_motor_signal(&motorA, 1, 10);
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					printf("Duty Cycle: 10\r\n");
					set_drive_motor_signal(&motorA, 1, 25);
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					printf("Duty Cycle: 25\r\n");
					set_drive_motor_signal(&motorA, 1, 50);
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					printf("Duty Cycle: 50\r\n");
					set_drive_motor_signal(&motorA, 1, 85);
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					cyhal_system_delay_ms(3000);
					printf(".\r\n");
					printf("Duty Cycle: 85\r\n");
					set_drive_motor_signal(&motorA, 1, 100);
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					printf("Ending motor signal 1A.\r\n");
					set_drive_motor_signal(&motorA, 1, 0);
				}
				else if(strncmp(sig_str, "2a", 2) == 0){
					printf("==============================================\r\n");
					printf("Sending motor signal 2A, 20Hz for 10 seconds.\r\n");
					set_drive_motor_signal(&motorA, 2, 10);
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					cyhal_system_delay_ms(1000);
					printf(".\r\n");
					printf("Ending motor signal 2A.\r\n");
					set_drive_motor_signal(&motorA, 2, 0);
				}
				else if(strncmp(sig_str, "1b", 2) == 0){
					printf("==============================================\r\n");
					printf("Sending motor signal 1B, 20Hz for 10 seconds.\r\n");
					set_drive_motor_signal(&motorB, 1, 10);
					printf("[");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..]\r\n");
					printf("Ending motor signal 1B.\r\n");
					set_drive_motor_signal(&motorB, 1, 0);
				}
				else if(strncmp(sig_str, "2b", 2) == 0){
					printf("==============================================\r\n");
					printf("Sending motor signal 2B, 20Hz for 10 seconds.\r\n");
					set_drive_motor_signal(&motorB, 2, 10);
					printf("[");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..");
					cyhal_system_delay_ms(1000);
					printf("..]\r\n");
					printf("Ending motor signal 2B.\r\n");
					set_drive_motor_signal(&motorB, 2, 0);
				}
				else{
					printf("No motor signal specified. enter signal name after CMD. 1a, 2a, 1b, 2b");
				}

			} else if (strncmp(pcInputString, "motorA ", 7) == 0){
				char sig_str[2];
				sig_str[0] = pcInputString[7];
				sig_str[1] = pcInputString[8];
				printf("Current Motor Status\r\n");
				print_motor(&motorA);
				
				if(strncmp(sig_str, "50", 2) == 0){
					set_drive_motor_speed(&motorA, 50);
					print_motor(&motorA);
				}
				else if(strncmp(sig_str, "cc", 2) == 0){
					set_drive_motor_direction(&motorA, 1);
					print_motor(&motorA);
				}
				else if(strncmp(sig_str, "cw", 2) == 0){
					set_drive_motor_direction(&motorA, -1);
					print_motor(&motorA);
				}
				else {
					set_drive_motor_speed(&motorA, 0);
					set_drive_motor_signal(&motorA, 1, 0);
					set_drive_motor_signal(&motorA, 2, 0);
					print_motor(&motorA);
				}
			} else {
				printf("CMD fail: command not recognized.\r\n");
			}
			cInputIndex = 0;
			ALERT_CONSOLE_RX = false;
		}
    }
	//motorfree();
}



/* [] END OF FILE */
