/*
 * opt3001.c
 *
 *  Created on: Oct 20, 2020
 *      Author: Joe Krachey
 */

#include "i2c.h"
#include "LM75.h"
#include "result_tools.h"

/** Write a register on the TCA9534
 *
 * @param reg The reg address to read
 * @param value The value to be written
 *
 */
void LM75_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t write_buffer[2];

	/* ADD CODE */
	write_buffer[0] = reg;
	write_buffer[1] = value;

	/* ADD CODE */
	/* Use cyhal_i2c_master_write to write the required data to the device. */
	cy_rslt_t result = cyhal_i2c_master_write(&i2c_master_obj, 				// I2C Object
							LM75_SUBORDINATE_ADDR,		// I2C Address
							write_buffer, 					// Array of data to write
							2, 								// Number of bytes to write
							10000, 							// Wait 10s
							true);							// Generate Stop Condition

	if (result != CY_RSLT_SUCCESS) {
		print_result(result);
	}

}

/** Read a register on the TCA9534
 *
 * @param reg The reg address to read
 *
 */
static int16_t LM75_read_reg(uint8_t reg)
{
	uint8_t write_buffer[1];
	uint8_t read_buffer[2];
	int16_t return_value;

	write_buffer[0] = reg;
	
	/* Send the register address, do not generate a stop condition.  This will result in */
	/* a restart condition. */
	cy_rslt_t result = cyhal_i2c_master_write(
							&i2c_master_obj,
							LM75_SUBORDINATE_ADDR,	// I2C Address
							write_buffer, 					// Array of data to write
							1, 								// Number of bytes to write
							10000, 							// Wait 10s
							false);							// Do NOT generate Stop Condition

	if (result == CY_RSLT_SUCCESS) {
		// The register address has already been set in the write above, so read a single byte
		// of data.
		cy_rslt_t result = cyhal_i2c_master_read(
								&i2c_master_obj, 			    // I2C Object
								LM75_SUBORDINATE_ADDR,	// I2C Address
								read_buffer, 					// Read Buffer
								2 , 							// Number of bytes to read
								10000, 							// Wait 10s
								true);
		if (result == CY_RSLT_SUCCESS) 							// Generate Stop Condition
		{

			// Shift bits 9-1 of the temperature by 1
			return_value = (int16_t)(read_buffer[0] << 1);

			// Add in bit 0 of the temperature if it is non-zero
			if(read_buffer[1] & 0x80)
			{
				return_value += 0x01;
			}

			// Set the sign bits of bits 15-10
			if( return_value & 0x0100)
			{
				return_value = return_value | 0xFE;
			}

			return return_value;
		}
		else
		{
			print_result(result);

		     /* Disable all interrupts. */
		    __disable_irq();

		    CY_ASSERT(0);

		    while(1){};
		}
	}
	else
	{
		print_result(result);

	     /* Disable all interrupts. */
	    __disable_irq();

	    CY_ASSERT(0);

	    while(1){};
	}

	return 0xFF; // Should never get here
}



/** Read the value of the input port
 *
 * @param reg The reg address to read
 *
 */
float LM75_get_temp(void)
{
	int16_t raw_value = LM75_read_reg(LM75_TEMP_REG);
	float temp = (double)raw_value / 2;
	return temp;
}
