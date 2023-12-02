#include "ir.h"

VL53LX_Dev_t IR_dev[NUM_IR];

void ir_boot() {
    for (int i = 0; i < NUM_IR; i++) {
        cy_rslt_t result = ir_mux_set_chnl(i);
        if (result != CY_RSLT_SUCCESS) {
            printf("IR Error: Unable to set IR channel to %i, reason:\n\r", i);
            print_result(result);
        }
        bool ir_wait_boot = true;
        if (ir_wait_boot) {
            VL53LX_Dev_t boot_dev = IR_dev[i];
            printf("* -- Waiting for IR %i boot\n\r", i);
            VL53LX_Error err = VL53LX_WaitDeviceBooted(&boot_dev);
            if (err != VL53LX_ERROR_NONE) {
                printf("IR Error: Unable to detect IR %i boot, reason:\r\n", i);
                print_IR_error(err);
                continue;
            } else {
                printf("* -- Initializing IR %i data\n\r", i);
                VL53LX_Error err = VL53LX_DataInit(&boot_dev);
                if (err != VL53LX_ERROR_NONE) {
                    printf("CMD fail: Unable to initialize IR %i data, reason:\r\n", i);
                    print_IR_error(err);
                    continue;
                } else {
                    printf("* -- Starting IR %i measurement\n\r", i);
                    err = VL53LX_StartMeasurement(&boot_dev);
                    if (err != VL53LX_ERROR_NONE) {
                        printf("CMD fail: Unable to start IR %i, reason:\r\n", i);
                        print_IR_error(err);
                        continue;
                    }
                }
            }
            IR_dev[i] = boot_dev;
        }
    }
}

void ir_io_test(int dev_num) {
    uint8_t byteData;
    uint16_t wordData;
    
    cy_rslt_t result = ir_mux_set_chnl(dev_num);
    if (result != CY_RSLT_SUCCESS) {
        printf("IR Error: Couldn't switch mux to IR %i, reason:\r\n", dev_num);
        print_result(result);
        return;
    }
    VL53LX_Dev_t test_dev = IR_dev[dev_num];

    VL53LX_Error err = VL53LX_RdByte(&test_dev, 0x010F, &byteData);
    if (err != VL53LX_ERROR_NONE) {
        printf("IR Error: Couldn't read ID from IR %i, reason:\r\n", dev_num);
        print_IR_error(err);
        return;
    }
    printf("VL53LX %i Model_ID: %02X\n\r", dev_num, byteData);

    VL53LX_RdByte(&test_dev, 0x0110, &byteData);
    if (err != VL53LX_ERROR_NONE) {
        printf("IR Error: Couldn't read ID from IR %i, reason:\r\n", dev_num);
        print_IR_error(err);
        return;
    }
    printf("VL53LX %i Module_Type: %02X\n\r", dev_num, byteData);

    VL53LX_RdWord(&test_dev, 0x010F, &wordData);
    if (err != VL53LX_ERROR_NONE) {
        printf("IR Error: Couldn't read ID from IR %i, reason:\r\n", dev_num);
        print_IR_error(err);
        return;
    }
    printf("VL53LX %i: %02X\n\r", dev_num, wordData);

    //i2c_test(&test_dev);
    IR_dev[dev_num] = test_dev;
}

void ir_read_all(int num_measurements, bool verbose) {
    for (int i = 0; i < NUM_IR; i++) {
        ir_read(i, num_measurements, verbose);
    }
}

void ir_read(int dev_num, int num_measurements, bool verbose) {
    VL53LX_Dev_t measure_dev = IR_dev[dev_num];
    cy_rslt_t result = ir_mux_set_chnl(dev_num);
    if (result != CY_RSLT_SUCCESS) {
        print_result(result);
        printf("IR Error: Couldn't test IR %i, see error above\r\n", dev_num);
        return;
    }
    if (verbose) {
        printf("IR Info: Starting IR %i measurement 1 of %i...\r\n", dev_num, num_measurements);
    }
    VL53LX_Error err = VL53LX_ClearInterruptAndStartMeasurement(&measure_dev);
    if (err != VL53LX_ERROR_NONE) {
        printf("IR Error: Failed to start a measurement, reason:\r\n");
        print_IR_error(err);
    } else {
        for (int i = 2; i < num_measurements + 2; i++) {
            if (verbose) {
                printf("IR Info: IR measurement started. Waiting until ready...\r\n");
            }
            err = VL53LX_WaitMeasurementDataReady(&measure_dev);
            if (err != VL53LX_ERROR_NONE) {
                printf("IR Error: Failed to wait for a measurement, reason:\r\n");
                print_IR_error(err);
            } else {
                VL53LX_MultiRangingData_t data;
                err = VL53LX_GetMultiRangingData(&measure_dev, &data);
                if (err != VL53LX_ERROR_NONE) {
                    printf("IR Error: Failed to get measurement data, reason:\r\n");
                    print_IR_error(err);
                } else {
                    if (verbose) {
                        printf("IR Info: Furthest object is %i mm away\r\n", data.RangeData->RangeMaxMilliMeter);
                        printf("IR Info: Closest object is %i mm away\r\n", data.RangeData->RangeMinMilliMeter);
                        printf("IR Info: IR sees %i objects\r\n", data.NumberOfObjectsFound);
                        printf("IR Info: Status: %i\r\n", data.RangeData->RangeStatus);
                    }
                }
            }
            if (i < (num_measurements + 1)) {
                if (verbose) {
                    printf("IR Info: Starting IR %i measurement %i of %i...\r\n", dev_num, i, num_measurements);
                }
                VL53LX_Error err = VL53LX_ClearInterruptAndStartMeasurement(&measure_dev);
                if (err != VL53LX_ERROR_NONE) {
                    printf("IR Error: Failed to start a measurement, reason:\r\n");
                    print_IR_error(err);
                }
            }
        }
    }
    IR_dev[dev_num] = measure_dev;
}