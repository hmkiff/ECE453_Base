/*
 * imu.c
 *
 *  Created on: Oct 10, 2023
 *      Author: Ryan Almizyed
 */

#ifndef IMU_H_
#define IMU_H_

// EEPROM includes
#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "console.h"
#include "spi.h"

/* Provided Includes ---------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "lsm6dsm_reg.h"

/* Private constants -------------------------------------------------------------*/
#define IMU_CS_PIN P5_3
// #define IMU_CS_PIN P10_5

#define IMU_RW_READ_SET 0x80
#define IMU_RW_WRITE_MASK 0x7F

#define IMU_CTRL3_C 0x76

/* Extern variables ----------------------------------------------------------*/


/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */

void imu_orientation(void);
cy_rslt_t imu_cs_init(void);

#endif
