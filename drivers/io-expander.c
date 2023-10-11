#include "i2c.h"
#include "io-expander.h"
#include "../debug-tools/result_tools.h"

void io_expander_write_reg(uint8_t reg, uint8_t value) {
    uint8_t write_buffer[2];

    write_buffer[0] = reg;
	write_buffer[1] = value;

	cy_rslt_t result = cyhal_i2c_master_write(&i2c_master_obj, 	// I2C Object
							IO_EXPANDER_SUBORDINATE_ADDR,	// I2C Address
							write_buffer, 					// Reg addr
							2, 								// Number of bytes to write
							10000, 							// Wait 10s
							true);							// Generate Stop Condition

	if (result != CY_RSLT_SUCCESS) {
		print_result(result);
        CY_ASSERT(0);
        while(1) {}
	}
}