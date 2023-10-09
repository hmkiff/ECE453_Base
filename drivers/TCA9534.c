#include "i2c.h"
#include "console.h"
#include "../debug-tools/result_tools.h"

#define TCA9534_ADDR 0x01

#define TCA9534_IN_CMD 0x00
#define TCA9534_OUT_CMD 0x01
#define TCA9534_POLARITY_INVERSION_CMD 0x02
#define TCA9534_CONFIG_CMD 0x03

int TCA9534_get_input_port() {
    return -1; // Not implemented
}

cy_rslt_t TCA9534_send_command(uint8_t cmd_byte, uint8_t data_byte) {
    cy_rslt_t result;

    const uint8_t* cmd_byte_ptr = &cmd_byte;
    result = cyhal_i2c_master_write(&i2c_master_obj, TCA9534_ADDR, cmd_byte_ptr, 8, 1000, 0);
    if(result != CY_RSLT_SUCCESS)
	{
        return result;
	}

    const uint8_t* data_byte_ptr = &data_byte;
    result = cyhal_i2c_master_write(&i2c_master_obj, TCA9534_ADDR, data_byte_ptr, 8, 1000, 1);

    return result;
}

/**
 * @brief Sets ports as inputs or outputs by bit mask
 * 
 * @param mask Bit mask. 0s are output, 1s are input.
 */
void TCA9534_set_configuration(int mask) {
    printf("* -- Setting TCA pin configuration\n\r");
    uint8_t cmd_byte = TCA9534_CONFIG_CMD;
    cy_rslt_t result = TCA9534_send_command(cmd_byte, mask);
    if (result != CY_RSLT_SUCCESS) {
        print_result(result);
        printf("* -- Setting TCA pin configuration failed.\n\r");
        CY_ASSERT(0);
        while (1) {}
    } else {
        printf("* -- Setting TCA pin configuration success.\n\r");
        CY_ASSERT(0);
        while (1) {}
    }
}

/**
 * @brief Sets the values of output ports by bit mask.
 * 
 * @param mask 
 */
void TCA9534_set_output_port(int mask) {
    printf("* -- Setting TCA outputs\n\r");
    uint8_t cmd_byte = TCA9534_OUT_CMD;
    if (TCA9534_send_command(cmd_byte, mask) != CY_RSLT_SUCCESS) {
        printf("* -- Setting TCA outputs failed.\n\r");
    }
}