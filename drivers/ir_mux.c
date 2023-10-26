#include "i2c.h"
#include "ir_mux.h"
#include "../drivers/console.h"

cy_rslt_t ir_mux_set_chnl(uint8_t chnl) {
    switch (chnl) {
        case 1: {
            chnl = 0b00000001;
            break;
        }
        case 2: {
            chnl = 0b00000010;
            break;
        }
        case 3: {
            chnl = 0b00000100;
            break;
        }
        case 4: {
            chnl = 0b00001000;
            break;
        }
        default: {
            printf("ir_mux error: Tried to set IR mux to an invalid channel.\n\r");
        }
    }

    uint8_t *write_buffer = &chnl;
	cy_rslt_t result = cyhal_i2c_master_write(&i2c_master_obj, 	// I2C Object
							IR_MUX_SUBORDINATE_ADDR,	    // I2C Address
							write_buffer, 					// Reg addr
							1, 								// Number of bytes to write
							I2C_TIMEOUT, 			        // Wait 
							true);							// Generate Stop Condition

    return result;
}