#ifndef RESULT_TOOLS_H_
#define RESULT_TOOLS_H_

#include "cy_result.h"
#include "drivers/VL53L3CX_API_1.2.8/vl53lx_error_codes.h"

void print_result(cy_rslt_t result);
void print_IR_error(VL53LX_Error error);

#endif