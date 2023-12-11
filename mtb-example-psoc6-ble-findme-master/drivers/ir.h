#ifndef IR_H_
#define IR_H_

#include "cy_pdl.h"
#include "cybsp.h"
#include "cyhal.h"
#include "vl53lx_api.h"
#include "ir_mux.h"
#include "result_tools.h"

#define NUM_IR 4

volatile extern VL53LX_Dev_t IR_dev[NUM_IR];
volatile extern VL53LX_MultiRangingData_t multi_ir_data_store[NUM_IR];

void ir_boot();
void ir_io_test(int dev_num);
void ir_read_all(int num_measurements, bool verbose);
void ir_read_until_valid(int dev_num, int max, bool verbose);
uint8_t ir_read(int dev_num, int num_measurements, bool verbose);

#endif