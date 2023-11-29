
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

// Was present in original platform file, but we can't use -MW
// #include <windows.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

#include "../i2c.h"
#include "../console.h"
#include "../debug-tools/result_tools.h"

#include "cyhal_timer.h"
#include "cyhal_syspm.h"

#include "vl53lx_platform.h"
#include "vl53lx_platform_log.h"

#define VL53LX_get_register_name(VL53LX_p_007,VL53LX_p_032) VL53LX_COPYSTRING(VL53LX_p_032, "");

// Timer used for polling
cyhal_timer_t IR_poll_timer;
uint8_t global_comms_type = VL53LX_I2C;

#define  VL53LX_COMMS_CHUNK_SIZE  56
#define  VL53LX_COMMS_BUFFER_SIZE 64

// Unused
#define GPIO_INTERRUPT          RS_GPIO62
#define GPIO_POWER_ENABLE       RS_GPIO60
#define GPIO_XSHUTDOWN          RS_GPIO61
#define GPIO_SPI_CHIP_SELECT    RS_GPIO51

#define IR_SERF_ADDR 0b0101001

VL53LX_Error VL53LX_CommsInitialise(
	VL53LX_Dev_t *pdev,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz) {

	VL53LX_Error status = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(pdev);
	SUPPRESS_UNUSED_WARNING(comms_speed_khz);

	global_comms_type = VL53LX_I2C;

	// Assume we already ran i2c_init

	return status;
}


VL53LX_Error VL53LX_CommsClose(
	VL53LX_Dev_t *pdev) {
		
	VL53LX_Error status = VL53LX_ERROR_NONE;

	SUPPRESS_UNUSED_WARNING(pdev);

	if (global_comms_type == VL53LX_I2C) {
		cy_rslt_t result = i2c_close();
		if (result != CY_RSLT_SUCCESS) {
			print_result(result);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}
	} else if (global_comms_type == VL53LX_SPI) {
		// SPI not ported
		printf("VL53LX_CommsInitialise Error: SPI not ported\n\r");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	} else {
		printf("VL53LX_CommsInitialise Error: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}



VL53LX_Error VL53LX_WriteMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count) {

	VL53LX_Error status = VL53LX_ERROR_NONE;

	if (global_comms_type == VL53LX_I2C) {	
		uint8_t index_buffer[2];
		index_buffer[0] = (uint8_t) (index >> 8);
		index_buffer[1] = (uint8_t) (index);

		/*
		cy_rslt_t result = cyhal_i2c_master_write(
			&i2c_master_obj,
			IR_SERF_ADDR,
			pdata,
			count,
			I2C_TIMEOUT,
			1
		);
		*/

		cyhal_i2c_t *obj = &i2c_master_obj;
		uint16_t dev_addr = IR_SERF_ADDR;
		const uint8_t *data = pdata;
		uint16_t size = count;
		uint32_t timeout = I2C_TIMEOUT;
		bool send_stop = 1;

		if (_cyhal_scb_pm_transition_pending())
        	return CYHAL_SYSPM_RSLT_ERR_PM_PENDING;

		cy_en_scb_i2c_status_t status = (obj->context.state == CY_SCB_I2C_IDLE)
			? Cy_SCB_I2C_MasterSendStart(obj->base, dev_addr, CY_SCB_I2C_WRITE_XFER, timeout, &obj->context)
			: Cy_SCB_I2C_MasterSendReStart(obj->base, dev_addr, CY_SCB_I2C_WRITE_XFER, timeout, &obj->context);

		if (status == CY_SCB_I2C_SUCCESS) {
			status = Cy_SCB_I2C_MasterWriteByte(obj->base, index_buffer[0], timeout, &obj->context);
			if (status != CY_SCB_I2C_SUCCESS) {
				print_result(status);
				return VL53LX_ERROR_CONTROL_INTERFACE;
			}
			status = Cy_SCB_I2C_MasterWriteByte(obj->base, index_buffer[1], timeout, &obj->context);
			if (status != CY_SCB_I2C_SUCCESS) {
				print_result(status);
				return VL53LX_ERROR_CONTROL_INTERFACE;
			}
			while (size > 0)
			{
				status = Cy_SCB_I2C_MasterWriteByte(obj->base, *data, timeout, &obj->context);
				if (status != CY_SCB_I2C_SUCCESS) {
					print_result(status);
					return VL53LX_ERROR_CONTROL_INTERFACE;
				}
				--size;
				++data;
			}
		}

		if (send_stop)
		{
			/* SCB in I2C mode is very time sensitive. In practice we have to request STOP after */
			/* each block, otherwise it may break the transmission */
			Cy_SCB_I2C_MasterSendStop(obj->base, timeout, &obj->context);
		}

		if (status != CY_RSLT_SUCCESS) {
			print_result(status);
			return VL53LX_ERROR_CONTROL_INTERFACE;
		}
	} else if (global_comms_type == VL53LX_SPI) {
		// SPI not ported
		printf("VL53LX_CommsInitialise Error: SPI not ported\n\r");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	} else {
		printf("VL53LX_CommsInitialise Error: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}

VL53LX_Error VL53LX_ReadMulti(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata,
	uint32_t      count)
{

	VL53LX_Error status = VL53LX_ERROR_NONE;

	if (global_comms_type == VL53LX_I2C) {
		uint8_t index_buffer[2];
		index_buffer[0] = (uint8_t) (index >> 8);
		index_buffer[1] = (uint8_t) (index);

		cy_rslt_t result = cyhal_i2c_master_write(
			&i2c_master_obj,
			IR_SERF_ADDR,
			index_buffer,
			2,
			I2C_TIMEOUT,
			1
		);
		if (result != CY_RSLT_SUCCESS) {
			print_result(result);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}

		result = cyhal_i2c_master_read(
			&i2c_master_obj,
			IR_SERF_ADDR,
			pdata,
			count,
			I2C_TIMEOUT,
			1
		);
		if (result != CY_RSLT_SUCCESS) {
			print_result(result);
			status = VL53LX_ERROR_CONTROL_INTERFACE;
		}

	} else if (global_comms_type == VL53LX_SPI) {
		// SPI not ported
		printf("VL53LX_CommsInitialise Error: SPI not ported\n\r");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	} else {
		printf("VL53LX_CommsInitialise Error: Comms must be one of VL53LX_I2C or VL53LX_SPI\n");
		status = VL53LX_ERROR_CONTROL_INTERFACE;
	}

	return status;
}


VL53LX_Error VL53LX_WrByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t       VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[1];

	buffer[0] = (uint8_t)(VL53LX_p_003);

	status = VL53LX_WriteMulti(pdev, index, buffer, 1);

	return status;
}


VL53LX_Error VL53LX_WrWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	buffer[0] = (uint8_t)(VL53LX_p_003 >> 8);
	buffer[1] = (uint8_t)(VL53LX_p_003 &  0x00FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

	return status;
}


VL53LX_Error VL53LX_WrDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t      VL53LX_p_003)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[4];

	buffer[0] = (uint8_t) (VL53LX_p_003 >> 24);
	buffer[1] = (uint8_t)((VL53LX_p_003 &  0x00FF0000) >> 16);
	buffer[2] = (uint8_t)((VL53LX_p_003 &  0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (VL53LX_p_003 &  0x000000FF);

	status = VL53LX_WriteMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

	return status;
}


VL53LX_Error VL53LX_RdByte(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint8_t      *pdata)
{
	VL53LX_Error status = VL53LX_ERROR_NONE;
	status = VL53LX_ReadMulti(pdev, index, pdata, 1);
	return status;
}


VL53LX_Error VL53LX_RdWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint16_t     *pdata)
{
	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint8_t  buffer[2];

	status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_WORD);

	*pdata = (uint16_t)(((uint16_t)(buffer[0])<<8) + (uint16_t)buffer[1]);

	return status;
}

VL53LX_Error VL53LX_RdDWord(
	VL53LX_Dev_t *pdev,
	uint16_t      index,
	uint32_t     *pdata) {

	VL53LX_Error status = VL53LX_ERROR_NONE;
	uint8_t buffer[4];

	status = VL53LX_ReadMulti(pdev, index, buffer, VL53LX_BYTES_PER_DWORD);

	*pdata = ((uint32_t)buffer[0]<<24) + ((uint32_t)buffer[1]<<16) + ((uint32_t)buffer[2]<<8) + (uint32_t)buffer[3];

	return status;
}

VL53LX_Error VL53LX_WaitUs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_us) {

	Cy_SysLib_DelayUs(wait_us);
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_WaitMs(
	VL53LX_Dev_t *pdev,
	int32_t       wait_ms) {

	Cy_SysLib_Delay(wait_ms);
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerFrequency(int32_t *ptimer_freq_hz) {
	*ptimer_freq_hz = 0;

	printf("VL53LX_GetTimerFrequency: Freq : %ldHz\n", *ptimer_freq_hz);
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GetTimerValue(int32_t *ptimer_count) {
	*ptimer_count = 0;

	printf("VL53LX_GetTimerValue: Freq : %ldHz\n", *ptimer_count);
	return VL53LX_ERROR_NONE;
}

VL53LX_Error VL53LX_GpioSetMode(uint8_t pin, uint8_t mode) {
	printf("VL53LX_GpioSetMode: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}


VL53LX_Error  VL53LX_GpioSetValue(uint8_t pin, uint8_t value) {
	printf("VL53LX_GpioSetValue: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioGetValue(uint8_t pin, uint8_t *pvalue) {
	printf("VL53LX_GpioGetValue: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioXshutdown(uint8_t value) {
	printf("VL53LX_GpioXshutdown: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error VL53LX_GpioCommsSelect(uint8_t value) {
	printf("VL53LX_GpioCommsSelect: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error  VL53LX_GpioPowerEnable(uint8_t value) {
	printf("VL53LX_GpioPowerEnable: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error  VL53LX_GpioInterruptEnable(void (*function)(void), uint8_t edge_type) {
	printf("VL53LX_GpioInterruptEnable: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

VL53LX_Error  VL53LX_GpioInterruptDisable(void) {
	printf("VL53LX_GpioInterruptEnable: Not implemented");
	return VL53LX_ERROR_CONTROL_INTERFACE;
}

void VL53LX_StartTickCount() {
	cy_rslt_t rslt = cyhal_timer_init(&IR_poll_timer, NC, NULL);
	if (rslt != CY_RSLT_SUCCESS) {
		printf("IR error: Unable to initialize IR polling timer, see error below\r\n");
		print_result(rslt);
		return;
	}

	cyhal_timer_cfg_t cfg;
	cfg.is_continuous = false;
	cfg.is_compare = false;
	cfg.value = 0;
	cfg.direction = CYHAL_TIMER_DIR_UP;
	cfg.period = 5000;

	rslt = cyhal_timer_configure(&IR_poll_timer, &cfg);
	if (rslt != CY_RSLT_SUCCESS) {
		printf("IR error: Unable to configure IR polling timer, see error below\r\n");
		print_result(rslt);
		return;
	}

	rslt = cyhal_timer_set_frequency(&IR_poll_timer, 1000);
	if (rslt != CY_RSLT_SUCCESS) {
		printf("IR error: Unable to set frequency of IR polling timer, see error below\r\n");
		print_result(rslt);
		return;
	}

	rslt = cyhal_timer_start(&IR_poll_timer);
	if (rslt != CY_RSLT_SUCCESS) {
		printf("IR error: Unable to start IR polling timer, see error below\r\n");
		print_result(rslt);
		return;
	}
}

void VL53LX_EndTickCount() {
	cyhal_timer_free(&IR_poll_timer);
}

VL53LX_Error VL53LX_GetTickCount(
		VL53LX_Dev_t *pdev,
		uint32_t *ptick_count_ms) {

	VL53LX_Error status  = VL53LX_ERROR_NONE;
	(void) pdev;

	*ptick_count_ms = cyhal_timer_read(&IR_poll_timer);

	return status;
}


VL53LX_Error VL53LX_WaitValueMaskEx(
	VL53LX_Dev_t *pdev,
	uint32_t      timeout_ms,
	uint16_t      index,
	uint8_t       value,
	uint8_t       mask,
	uint32_t      poll_delay_ms)
{


	VL53LX_Error status         = VL53LX_ERROR_NONE;
	uint32_t     start_time_ms   = 0;
	uint32_t     current_time_ms = 0;
	uint8_t      byte_value      = 0;
	uint8_t      found           = 0;

	SUPPRESS_UNUSED_WARNING(poll_delay_ms);

	VL53LX_StartTickCount();
	VL53LX_GetTickCount(pdev, &start_time_ms);
	pdev->new_data_ready_poll_duration_ms = 0;

	while ((status == VL53LX_ERROR_NONE) &&
		   (pdev->new_data_ready_poll_duration_ms < timeout_ms) &&
		   (found == 0))
	{
		status = VL53LX_RdByte(
						pdev,
						index,
						&byte_value);

		if ((byte_value & mask) == value)
		{
			found = 1;
		}

		VL53LX_GetTickCount(pdev, &current_time_ms);
		printf("IR polling time is %li ms.\r\n", current_time_ms);
		pdev->new_data_ready_poll_duration_ms = current_time_ms - start_time_ms;
	}
	VL53LX_EndTickCount();

	if (found == 0 && status == VL53LX_ERROR_NONE) {
		status = VL53LX_ERROR_TIME_OUT;
	}

	return status;
}


