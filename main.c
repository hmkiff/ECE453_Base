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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#include "main.h"
#define ENABLE_I2C 0
#define ENABLE_SPI 1 //SPI disabled
//#define ENABLE_IMU 0

int main(void)
{
	float temp;
	uint8_t led_mask = 0x01;

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
    printf("****************** \r\n\n");

#if ENABLE_I2C
	io_expander_get_input_port();
	io_expander_set_configuration(0x00); 	// Set all pins as outputs
	io_expander_set_output_port(0x00); 		// Turn on all LEDs

#endif

    while(1)
    {
#if ENABLE_I2C
    	Cy_SysLib_Delay(1000);
    	temp = LM75_get_temp();
    	printf("Temperature = %.2f\r\n",temp);
    	io_expander_set_output_port(led_mask);

    	led_mask = led_mask << 1;
    	if(led_mask == 0x80)
    	{
    		led_mask = 0x01;
    	}
#endif
#if ENABLE_SPI
		//write SPI CLI implementation here
		//declare variables

    	Cy_SysLib_Delay(1000);
		uint16_t addr;
		//cy_rslt_t   rslt;
		uint8_t data;
		
		

		//initialize variables
		addr = 0xFFFF;
		data = 0xA2;

		//write to eeprom
		eeprom_write_byte(addr, data); //this function automatically writes and read for us
		
		//read eeprom = 
		eeprom_read_byte(addr, &data);
		
		eeprom_test() == CY_RSLT_SUCCESS; //set eeprom_test to SUCCESS

		
		

			


#endif
    }

//#if ENABLE_IMU
	//display current orientation of imu
//#endif
}



/* [] END OF FILE */
