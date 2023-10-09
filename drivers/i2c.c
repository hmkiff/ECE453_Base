/*
 * i2c.c
 *
 *  Created on: Jan 21, 2022
 *      Author: Joe Krachey
 */

#include "i2c.h"

cyhal_i2c_t i2c_master_obj;

// Define the I2C monarch configuration structure
cyhal_i2c_cfg_t i2c_master_config =
{
    CYHAL_I2C_MODE_MASTER,
    0, // address is not used for master mode
    I2C_MASTER_FREQUENCY
};

/** Initialize the I2C bus to the specified module site
 *
 * @param - None
 */
cy_rslt_t i2c_init(void)
{
	cy_rslt_t rslt;

    // Initialize I2C monarch, set the SDA and SCL pins and assign a new clock
	rslt = cyhal_i2c_init(&i2c_master_obj, PIN_MCU_SDA, PIN_MCU_SCL, NULL);

	if(rslt != CY_RSLT_SUCCESS)
	{
	    CY_ASSERT(0);

	    while(1){};
	}

    // Configure the I2C resource to be monarch
	rslt = cyhal_i2c_configure(&i2c_master_obj, &i2c_master_config) ;

	if(rslt != CY_RSLT_SUCCESS)
	{
	    CY_ASSERT(0);

	    while(1){};
	}

	return CY_RSLT_SUCCESS;
}
