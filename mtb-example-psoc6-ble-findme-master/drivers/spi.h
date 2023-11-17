/*
 *  Created on: Jan 18, 2022
 *      Author: Joe Krachey
 */

#ifndef SPI_H__
#define SPI_H__

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"

/* Macros */
#define PIN_MCU_SPI_MOSI	P5_0
#define PIN_MCU_SPI_MISO	P5_1
#define PIN_MCU_SPI_CLK		P5_2

#define SPI_FREQ			1000000

/* Public Global Variables */
extern cyhal_spi_t mSPI;



/* Public API */
cy_rslt_t spi_init(void);

#endif 
