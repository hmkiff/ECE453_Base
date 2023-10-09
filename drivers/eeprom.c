/*
 * eeprom.c
 *
 *  Created on: Jan 27, 2023
 *      Author: Joe Krachey
 */

#include "eeprom.h"

/** Initializes the IO pins used to control the CS of the
 *  EEPROM
 *
 * @param
 *
 */
cy_rslt_t eeprom_cs_init(void)
{
	/* Initialize RED LED*/
	return cyhal_gpio_init(
			EEPROM_CS_PIN,              // Pin
			CYHAL_GPIO_DIR_OUTPUT,      // Direction
			CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
			true);				        // InitialValue
}

/** Determine if the EEPROM is busy writing the last
 *  transaction to non-volatile storage
 *
 * @param
 *
 */
cy_rslt_t eeprom_wait_for_write(void)
{
	uint8_t     transmit_data[2] = { EEPROM_CMD_RDSR, 0xFF};
	uint8_t     receive_data[2] = {0x00, 0x00};
	cy_rslt_t   rslt;

	  // Check to see if the eeprom is still updating
	  // the data from the last write
	  do
	  {
		// Set the CS Low
		cyhal_gpio_write(EEPROM_CS_PIN, 0);

		// Starts a data transfer
		rslt = cyhal_spi_transfer(
				&mSPI,
				transmit_data,
				2u,
				receive_data,
				2u,
				0xFF
			);

		// Set the CS High
		cyhal_gpio_write(EEPROM_CS_PIN, 1);

		if (rslt != CY_RSLT_SUCCESS)
		{
			return rslt;
		}


	    // If the address was not ACKed, try again.
	  } while ((receive_data[1] & 0x01) != 0);

	  return CY_RSLT_SUCCESS;
}

/** Enables Writes to the EEPROM
 *
 * @param
 *
 */
cy_rslt_t eeprom_write_enable(void)
{
	uint8_t     transmit_data[1] = {EEPROM_CMD_WREN};
	uint8_t     receive_data[1] = {0x00};
	cy_rslt_t   rslt;


	// Set the CS Low
	cyhal_gpio_write(EEPROM_CS_PIN, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
				&mSPI,
				transmit_data,
				1u,
				receive_data,
				1u,
				0xFF
			);

	// Set the CS High
	cyhal_gpio_write(EEPROM_CS_PIN, 1);

  return rslt;
}

/** Disable Writes to the EEPROM
 *
 * @param
 *
 */
cy_rslt_t eeprom_write_disable(void)
{
	uint8_t     transmit_data[1] = {EEPROM_CMD_WRDI};
	uint8_t     receive_data[1] = {0x00};
	cy_rslt_t   rslt;


	// Set the CS Low
	cyhal_gpio_write(EEPROM_CS_PIN, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
				&mSPI,
				transmit_data,
				1u,
				receive_data,
				1u,
				0xFF
			);

	// Set the CS High
	cyhal_gpio_write(EEPROM_CS_PIN, 1);

  return rslt;
}


/** Writes a single byte to the specified address
 *
 * @param address -- 16 bit address in the EEPROM
 * @param data    -- value to write into memory
 *
 */
cy_rslt_t eeprom_write_byte(uint16_t address, uint8_t data)
{
	uint8_t     transmit_data[4];
	uint8_t     receive_data[4];
	cy_rslt_t   rslt;

	// Wait for any outstanding writes to complete
	rslt = eeprom_wait_for_write();
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	// Enable writes to the eeprom
	rslt = eeprom_write_enable();
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	transmit_data[0] = EEPROM_CMD_WRITE;
	transmit_data[1] = (uint8_t)(address >> 8);
	transmit_data[2] = (uint8_t)address;
	transmit_data[3] = data;

	// Set the CS Low
	cyhal_gpio_write(EEPROM_CS_PIN, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
			&mSPI,
			transmit_data,
			4u,
			receive_data,
			4u,
			0xFF
		);

	// Set the CS High
	cyhal_gpio_write(EEPROM_CS_PIN, 1);

	// Disable writes to the eeprom
	rslt = eeprom_write_disable();
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	return CY_RSLT_SUCCESS;
}

/** Reads a single byte to the specified address
 *
 * @param address -- 16 bit address in the EEPROM
 * @param data    -- value read from memory
 *
 */
cy_rslt_t eeprom_read_byte(uint16_t address, uint8_t *data)
{
	uint8_t     transmit_data[4];
	uint8_t     receive_data[4];
	cy_rslt_t   rslt;

	// Wait for any outstanding writes to complete
	rslt = eeprom_wait_for_write();
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	// Enable writes to the eeprom
	rslt = eeprom_write_enable();
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	transmit_data[0] = EEPROM_CMD_READ;
	transmit_data[1] = (uint8_t)(address >> 8);
	transmit_data[2] = (uint8_t)address;
	transmit_data[3] = 0x00;

	// Set the CS Low
	cyhal_gpio_write(EEPROM_CS_PIN, 0);

	// Starts a data transfer
	rslt = cyhal_spi_transfer(
			&mSPI,
			transmit_data,
			4u,
			receive_data,
			4u,
			0xFF
		);

	// Set the CS High
	cyhal_gpio_write(EEPROM_CS_PIN, 1);

	// Return the value from the EEPROM to the user
	*data = receive_data[3];

	// Disable writes to the eeprom
	rslt = eeprom_write_disable();
	if(rslt != CY_RSLT_SUCCESS)
	{
		return rslt;
	}

	return CY_RSLT_SUCCESS;
}


/** Tests Writing and Reading the EEPROM
 *
 * @param
 *
 */
cy_rslt_t eeprom_test(void)
{
	uint8_t i;
	uint16_t addr;
	cy_rslt_t   rslt;
	uint8_t data;
	uint8_t expected_data;

	// Write the data to the eeprom.
	addr = 0x20;
	data = 0x10;
	for(i = 0; i < 20; i++)
	{
		rslt = eeprom_write_byte(addr, data);
		if(rslt != CY_RSLT_SUCCESS)
		{
			printf("* -- EEPROM WRITE FAILURE\n\r");
			return -1;
		}
		addr++;
		data++;
	}

	// Read the data back and verify everything matches what was written
	addr = 0x20;
	expected_data = 0x10;
	for(i = 0; i < 20; i++)
	{
		rslt = eeprom_read_byte(addr, &data);
		if(rslt != CY_RSLT_SUCCESS)
		{
			printf("* -- EEPROM READ FAILURE\n\r");
			return -1;
		}
		if(expected_data != data)
		{
			printf("* -- EEPROM READ DATA DOES NOT MATCH\n\r");
			return -1;
		}

		addr++;
		expected_data++;
	}
	printf("* -- EEPROM Test Passed\n\r");
	return CY_RSLT_SUCCESS;
}
