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

// Enables --------------------
#define ENABLE_SPI 0
// I2C device enables
#define ENABLE_I2C 1
#define ENABLE_TEMP 0
#define ENABLE_IO_EXPANDER 0
#define ENABLE_IR_MUX 1
// ----------------------------

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

// I2C-based devices init
#if ENABLE_I2C
    printf("* -- Initializing I2C Bus\n\r");
    i2c_init();
	#if ENABLE_IO_EXPANDER
		printf("* -- Initializing I/O Expander\n\r");
		io_expander_set_all_out();
	#endif
	#if ENABLE_IR_MUX
		printf("* -- Initializing IR Mux\n\r");
		//ir_mux_set_chnl(1);
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


printf("Initialization complete.\r\n\n");

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
				#if ENABLE_I2C
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
				#else
					printf("CMD fail: I2C not enabled.\r\n");
				#endif
			} else if (strncmp(pcInputString, "led off ", 8) == 0) {
				#if ENABLE_I2C
					#if ENABLE_IO_EXPANDER
						char led_num_str = pcInputString[8];
						int led_num = atoi(&led_num_str);
						if ((led_num >= 0) && (led_num <= 7)) {
							printf("CMD result: Turning LED %i off\r\n", led_num);
							led_mask = led_mask & ~(uint8_t)pow(2, led_num);
							io_expander_write_reg(0x01, led_mask);
							printf("CMD result: LED %i turned off.\r\n", led_num);
						} else {
							printf("CMD fail: No LED at %i\r\n", led_num);
						}
					#else
						printf("CMD fail: io expander not enabled.\r\n");
					#endif
				#else
					printf("CMD fail: I2C not enabled.\r\n");
				#endif
			} else if (strncmp(pcInputString, "IR sel ", 7) == 0) {
				#if ENABLE_I2C
					#if ENABLE_IR_MUX
						char ch_num_str = pcInputString[7];
						int ch_num = atoi(&ch_num_str);
						if ((ch_num >= 1) && (ch_num <= 4)) {
							cy_rslt_t result = ir_mux_set_chnl(ch_num);
							if (result != CY_RSLT_SUCCESS) {
								print_result(result);
								printf("CMD fail: Couldn't switch IR, see error above\r\n");
							} else {
								printf("CMD result: IR swicthed to ch. %i\r\n", ch_num);
							}
						} else {
							printf("CMD fail: No IR at %i\r\n", ch_num);
						}
					#else
						printf("CMD fail: io expander not enabled.\r\n");
					#endif
				#else
					printf("CMD fail: I2C not enabled.\r\n");
				#endif
			} else {
				printf("CMD fail: command not recognized.\r\n");
			}
			cInputIndex = 0;
			ALERT_CONSOLE_RX = false;
		}
    }
}



/* [] END OF FILE */
