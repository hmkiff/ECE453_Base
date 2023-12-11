/*
 ******************************************************************************
 * @file    orientation_6d.c
 * @author  Sensors Software Solution Team
 * @brief   This file show how to detect 6D orientation from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#include "imu.h"


/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI, rst;
static char tx_buffer[1000];
static stmdev_ctx_t dev_ctx;


/* Private functions ---------------------------------------------------------*/
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);


/* Main Example --------------------------------------------------------------*/
void imu_orientation(void) {

    int16_t linear[3];
    int16_t angles[3];
  
    lsm6dsm_acceleration_raw_get(&dev_ctx, linear);
    lsm6dsm_angular_rate_raw_get(&dev_ctx, angles);
  
    // Convert to real units
    float acceleration_scale = 4.0 / 32768.0;  // ±4 g range
    float angular_rate_scale = 500.0 / 32768.0; // ±500 dps range
  
    float ang_val[3];
    float lin_val[3];
    
    char direction[3]; 
  
    //calculate position and velocity
    float t_time = 1.0/208.0; //fixed time inmterval equal to date rate 
  
  
    float ini_lin_velocity = 0.0;
    float ini_lin_position = 0.0;
    float lin_velocity[3];
    float lin_position[3];
  
    float ini_ang_position = 0.0;
    float ang_position[3];
    //float ang_velocity[3] = ang_val[3];
    // Array to store direction strings
    // linear[0] = linear[0] * full scale range
    for (int i = 0; i < 3; ++i) {
        ang_val[i] = angles[i] * angular_rate_scale;
        ang_position[i] = ang_val[i] * t_time + ini_ang_position;
        
        ini_ang_position = ang_position[i];   
    }


    for (int i = 0; i < 3; ++i) {
        lin_val[i] = linear[i] * acceleration_scale;
        lin_velocity [i] = 9.81 * lin_val[i] * t_time + ini_lin_velocity;
        lin_position[i] = (ini_lin_velocity * t_time) + (0.5 * 9.81 * lin_val[i] * t_time * t_time) + ini_lin_position;

        ini_lin_position = lin_position[i];
        ini_lin_velocity = lin_velocity[i];
    }

    sprintf(tx_buffer, "Linear Velocity:\r\n"
                       "\tx: %.5f m/s\r\n"
                       "\ty: %.5f m/s\r\n"
                       "\tz: %.5f m/s\r\n"
                       "Angular Velocity:\r\n"
                       
                       "\tz: %.5f dps\r\n",
                       lin_velocity[0], lin_velocity[1], lin_velocity[2],
                        ang_val[2]);

    printf("%s", tx_buffer);

    sprintf(tx_buffer, 
            "Linear position:\r\n"
            "\tx: %.7f m\r\n"
            "\ty: %.7f m\r\n"
            "\tz: %.7f m\r\n"
            "Angular position:\r\n"
            "\tz: %.7f degrees\r\n",
            lin_position[0], lin_position[1], lin_position[2], ang_position[2]);

    printf("%s", tx_buffer);


//     //check orientation by setting threshold

//     float thres_hold_x = 0.0;
//     float thres_hold_y = 0.0;

//     //check left of right orientation
//     if(lin_val[0] > 0){
//         direction[0] = 'R'; // 'R' for positive values
//         thres_hold_x = lin_val[0];
//     }
//     else if (lin_val[0] < 0){
//         direction[0] = 'L'; // 'R' for positive values
//         thres_hold_x = lin_val[0];
//     }

//     //check forward or backward orientation
//     if(lin_val[1] > 0){
//         direction[1] = 'F'; // 'R' for positive values
//         thres_hold_x = lin_val[1];
//     }
//     else if (lin_val[1] < 0){
//         direction[1] = 'B'; // 'R' for positive values
//         thres_hold_y = lin_val[1];
//     }
//     // for( int j = 0; j < 2; ++j){
        
//     //     if (lin_val[] > thres_hold){

//     //         thres_hold = lin_val[j];
//     //     }
//     // }
//     // for (int j = 0; j < 2; ++j){
//     //     if (lin_val[0] > 0) {
//     //     direction[0] = 'R'; // 'R' for positive values
//     // } 
//     //     else {
//     //         direction[0] = 'L'; // 'L' for negative values
//     //      }

//     //      if (lin_val[1] > 0) {
//     //     direction[1] = 'R'; // 'R' for positive values
//     // } 
//     //     else {
//     //         direction[1] = 'L'; // 'L' for negative values
//     //      }
//     // }
  
}



// sprintf(tx_buffer, "Linear acceleration:\r\n"
//                       "\tx: %s g\r\n"
//                       "\ty: %s g\r\n"
//                       "\tz: %s g\r\n"
//                       "Angular acceleration:\r\n"
//                       "\tx: %.2f dps\r\n"
//                       "\ty: %.2f dps\r\n"
//                       "\tz: %.2f dps\r\n",
//                       direction[0], direction[1], lin_val[2],
//                       ang_val[0], ang_val[1], ang_val[2]);

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  // Set up initial address byte for data transfer
  uint8_t	address_byte = IMU_RW_WRITE_MASK & reg;   // RW AD6 AD5 AD4 AD3 AD2 AD1 AD0

	// Set the CS Low
	cyhal_gpio_write(IMU_CS_PIN, 0);

	// Send address byte to IMU
  cy_rslt_t result = cyhal_spi_transfer(
    &mSPI,
    &address_byte,
    1u,
    NULL,
    0u,
    0xFF
  );

  // Send payload bytes to IMU
  result = cyhal_spi_transfer(
    &mSPI,
    bufp,
    len,
    NULL,
    0u,
    0xFF
  );

  // Set the CS High
	cyhal_gpio_write(IMU_CS_PIN, 1);

  return result;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  // Set up initial address byte for data transfer
  uint8_t address_byte = IMU_RW_READ_SET | reg;   // RW AD6 AD5 AD4 AD3 AD2 AD1 AD0

	// Set the CS Low
	cyhal_gpio_write(IMU_CS_PIN, 0);

	// Send address byte to IMU
  cy_rslt_t result1 = cyhal_spi_transfer(
    &mSPI,
    &address_byte,
    1u,
    NULL,
    0u,
    0xFF
  );

  // Keep sending bytes to IMU to read data from IMU
  cy_rslt_t result2 = cyhal_spi_transfer(
    &mSPI,
    NULL,
    0u,
    bufp,
    len,
    0xFF
  );

  // Set the CS High
	cyhal_gpio_write(IMU_CS_PIN, 1);

  return result1 && result2;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  cyhal_system_delay_ms(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  // Set the IMU CS pin to the high (start with SPI disabled)
  cyhal_gpio_init(
    IMU_CS_PIN,
    CYHAL_GPIO_DIR_OUTPUT,
    CYHAL_GPIO_DRIVE_STRONG,
    true
  );

  // Set IMU control settings
  uint8_t setting_byte[1];
  setting_byte[0] = IMU_CTRL3_C;
  platform_write(NULL, 0x12, setting_byte, 1u);

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = NULL;
  /* Wait sensor boot time */
  platform_delay(15);
  /* Check device ID */
  lsm6dsm_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSM_ID){
    printf("IMU not found - got %i\r\n", whoamI);
    return;
  }

  /* Restore default configuration */
  lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsm_reset_get(&dev_ctx, &rst);
  } while (rst);


  /* Set GY Output Data Rate */
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_208Hz);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_208Hz);
  /* Set 4g full XL scale */
  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_4g);
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_500dps);

}

/** Initializes the IO pins used to control the CS of the
 *  IMU
 *
 * @param
 *
 */
cy_rslt_t imu_cs_init(void)
{
  platform_init();
	return CY_RSLT_SUCCESS;
}

void get_position(void){
    //imu_orientation()
    
}


void get_orientation(void){
    
    //imu_orientation();
}
