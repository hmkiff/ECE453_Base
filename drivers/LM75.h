/*
 * opt3001.h
 *
 *  Created on: Oct 20, 2020
 *      Author: Joe Krachey
 */

#ifndef LM75_H_
#define LM75_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

#define LM75_SUBORDINATE_ADDR                 0b1001000

#define LM75_TEMP_REG						  0x00

/**
 *
 *
 *
 */
cy_rslt_t LM75_init(void);

void LM75_write_reg(uint8_t reg, uint8_t value);

/** Read the temperature from LM75
 *
 * @param
 *
 */
float LM75_get_temp(void);

#endif /* LM75_H_ */
