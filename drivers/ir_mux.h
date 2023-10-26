#ifndef IR_MUX_H_
#define IR_MUX_H_

#include <stdint.h>
#include "cy_result.h"

#define IR_MUX_SUBORDINATE_ADDR 0b1110000

// Writes value to register at given address
cy_rslt_t ir_mux_set_chnl(uint8_t chnl);

#endif