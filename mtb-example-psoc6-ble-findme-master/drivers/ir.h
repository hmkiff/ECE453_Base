#ifndef IR_H_
#define IR_H_

#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "vl53lx_api.h"
#include "ir_mux.h"

#define NUM_IR 4

void ir_boot();
void ir_io_test(int dev_num);
void ir_read(int dev_num, int num_measurements, bool verbose);

#endif