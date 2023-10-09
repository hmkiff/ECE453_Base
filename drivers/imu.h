#ifndef IMU_H_
#define IMU_H_

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "console.h"
#include "spi.h"

#define IMU_CS_PIN  = P9_3
// Define the SPI interface pins
#define IMU_SPI_MOSI    CYBSP_SPI_MOSI
#define IMU_SPI_MISO    CYBSP_SPI_MISO
#define IMU_SPI_CLK     CYBSP_SPI_CLK
#define IMU_SPI_CS      CYBSP_SPI_SS

// Function to initialize the IMU
cy_rslt_t imu_init(void);

// Function to read orientation data from the IMU
cy_rslt_t imu_read_orientation(float *roll, float *pitch, float *yaw);


#endif