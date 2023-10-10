#ifndef IO_EXPANDER_H_
#define IO_EXPANDER_H_

#define IO_EXPANDER_SUBORDINATE_ADDR 0b0100001

void io_expander_write_reg(uint8_t reg, uint8_t value);

#endif