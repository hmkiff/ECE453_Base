#include "vl53lx_core.h"
#include "vl53lx_platform.h"

#include "result_tools.h"

// Tests vl53lx_platform implementation
// Was written for VL53L1,
// By John E Kvam, found on ST forums. Ported.
VL53LX_Error rd_write_verification( VL53LX_Dev_t *dev, uint16_t addr, uint32_t expected_value) {

	VL53LX_Error Status  = VL53LX_ERROR_NONE;

	uint8_t bytes[4],mbytes[4];
	uint16_t words[2];
	uint32_t dword;

	int i;

	VL53LX_ReadMulti(dev, addr, mbytes, 4);

	for (i=0; i<4; i++){ VL53LX_RdByte(dev, addr+i, &bytes[i]); }
	for (i=0; i<2; i++){ VL53LX_RdWord(dev, addr+i*2, &words[i]); }

	Status = VL53LX_RdDWord(dev, addr, &dword);

	printf("expected   = %8x,\n",expected_value);
	printf("read_multi = %2x, %2x, %2x, %2x\n", mbytes[0],mbytes[1],mbytes[2],mbytes[3]);
	printf("read_bytes = %2x, %2x, %2x, %2x\n", bytes[0],bytes[1],bytes[2],bytes[3]);
	printf("read words = %4x, %4x\n",words[0],words[1]);
	printf("read dword = %8x\n",dword);

	if((mbytes[0]<<24 | mbytes[1]<<16 | mbytes[2]<<8 | mbytes[3]) != expected_value) return (-1);
	if((bytes[0]<<24 | bytes[1]<<16 | bytes[2]<<8 | bytes[3]) != expected_value) return (-1);
	if((words[0]<<16 | words[1]) != expected_value) return (-1);
	if(dword != expected_value) return(-1);

	return Status;
}

#define REG 0x3A

void i2c_test(VL53LX_Dev_t *dev) {

	VL53LX_Error Status  = VL53LX_ERROR_NONE;

	int err_count = 0;

	uint8_t buff[4] = {0x11,0x22,0x33,0x44};

	uint8_t long_out[135] ={0x29, 0x02, 0x10, 0x00, 0x22, 0xBC, 0xCC, 0x81, 0x80, 0x07, 0x16, 0x00, 0xFF, 0xFD,
							0xF7, 0xDE, 0xFF, 0x0F, 0x00, 0x15, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x44, 0x00, 0x2C, 0x00, 0x11, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
							0x00, 0x11, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF,
							0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x14, 0x21, 0x00, 0x00,
							0x02, 0x00, 0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x01, 0x00, 0x02, 0x00,
							0x9D, 0x07, 0x00, 0xD2, 0x05, 0x01, 0x68, 0x00, 0xC0, 0x08, 0x38, 0x00, 0x00, 0x00, 0x00, 0x03,
							0xB6, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x07, 0x05, 0x06, 0x06, 0x01, 0x00, 0x02,
							0xC7, 0xFF, 0x8B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40};

	uint8_t long_in[135]= {0xff};

  	int i=0;

    VL53LX_Error err;
	err = rd_write_verification(dev, 0x10f, 0xeaaa10ff);			// verify the Chip ID works
    if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: ID test failed, reason:\r\n");
        print_IR_error(err);
    }

	err = VL53LX_WriteMulti(dev, 0x01, long_out, 135);			// check if WriteMulti can write 135 bytes
    if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: WriteMulti test failed, reason:\r\n");
        print_IR_error(err);
    }

	err = VL53LX_ReadMulti(dev, 0x01, long_in, 135);			// check if WriteMulti can read 135 bytes
    if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: ReadMulti test failed, reason:\r\n");
        print_IR_error(err);
    }

	for (i = 0; i < 135; i++) {
        if (long_in[i] != long_out[i]) {
            printf("IR test error: Long I/O test failed, read %i did not match write %i at index %i\r\n", long_in[i], long_out[i], i);
        }
    }

	if (err_count > 10) Status++;

	err = VL53LX_WriteMulti(dev, REG,  buff, 4);				// check WriteMulti
    if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: WriteMulti test failed, reason:\r\n");
        print_IR_error(err);
    }
	if (rd_write_verification(dev, REG, 0x11223344) < 0) {
        printf("IR test error: WriteMulti verification failed\r\n");
    }

	err = VL53LX_WrDWord(dev, REG, 0xffeeddcc);				// check WrDWord
    if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: WrWord test failed, reason:\r\n");
        print_IR_error(err);
    }
	if (rd_write_verification(dev, REG, 0xffeeddcc) < 0) {
        printf("IR test error: WrWord verification failed\r\n");
    }

	err = VL53LX_WrWord(dev, REG, 0x5566);					// check WrWord
    if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: WrWord test failed, reason:\r\n");
        print_IR_error(err);
    }
	err = VL53LX_WrWord(dev, REG+2, 0x7788);
        if (err != VL53LX_ERROR_NONE) {
        printf("IR test error: WrWord test failed, reason:\r\n");
        print_IR_error(err);
    }

	if (rd_write_verification(dev, REG, 0x55667788) < 0) {
        printf("IR test error: WrWord verification failed\r\n");
    }

	for (i = 0; i < 4; i++){  
        VL53LX_WrByte (dev, REG+i, buff[i]); 
    }
    if (rd_write_verification(dev, REG,0x11223344) <0) {
        printf("IR test error: WrWord verification failed\r\n");
    }
}