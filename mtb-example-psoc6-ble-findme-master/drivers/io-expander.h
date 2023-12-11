#ifndef IO_EXPANDER_H_
#define IO_EXPANDER_H_

#include <stdint.h>

#define IO_EXPANDER_SUBORDINATE_ADDR 0b0100001

// Sets all pins to outputs
void io_expander_set_all_out();

// Writes value to register at given address
void io_expander_write_reg(uint8_t reg, uint8_t value);

#endif