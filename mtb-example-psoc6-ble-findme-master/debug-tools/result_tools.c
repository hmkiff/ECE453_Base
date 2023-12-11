#include "result_tools.h"
#include "../drivers/console.h"
#include "cyhal_general_types.h"
#include "cyhal_dac.h"
#include "cyhal_i2c.h"

void print_result(cy_rslt_t result) {
    cy_rslt_decode_t decoded_rslt;
    decoded_rslt.raw = result;

    char resultstr[128];
    for (int i = 0; i < 64; i++) {
        resultstr[i] = '\0';
    }

    cy_en_rslt_type_t type = decoded_rslt.type;
    char typestr[16];
    if (type == CY_RSLT_TYPE_ERROR) {
        strcpy(typestr, "Error");
    } else if (type == CY_RSLT_TYPE_FATAL) {
        strcpy(typestr,"Fatal");
    } else if (type == CY_RSLT_TYPE_INFO) {
        strcpy(typestr, "Info");
    } else if (type == CY_RSLT_TYPE_WARNING) {
        strcpy(typestr, "Warning");
    } else {
        strcpy(typestr, "Unknown");
    }

    cy_en_rslt_module_t module = decoded_rslt.module;
    uint16_t code = decoded_rslt.code;
    char modulestr[16];
    char codestr[64];
    if (module == CYHAL_RSLT_MODULE_ADC) {
        strcpy(modulestr, "ADC");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_CLOCK) {
        strcpy(modulestr, "Clock");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_COMP) {
        strcpy(modulestr, "Comparator\n");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_CRC) {
        strcpy(modulestr, "Crypto CRC");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_DAC) {
        strcpy(modulestr, "DAC");
        if (code == CYHAL_DAC_RSLT_BAD_ARGUMENT) {
            strcpy(codestr, "Bad argument");
        } else if (code == CYHAL_DAC_RSLT_FAILED_INIT) {
            strcpy(codestr, "Failed init");
        } else if (code == CYHAL_DAC_RSLT_BAD_REF_VOLTAGE) {
            strcpy(codestr, "Bad reference voltage");
        } else if (code == CYHAL_DAC_RSLT_BAD_OPAMP_INSTANCE) {
            strcpy(codestr, "Bad opamp instance");
        } else {
            strcpy(codestr, "Unknown error");
        }
    } else if (module == CYHAL_RSLT_MODULE_DMA) {
        strcpy(modulestr, "DMA");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_EZI2C) {
        strcpy(modulestr, "EZI2C");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_FLASH) {
        strcpy(modulestr, "Flash");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_GPIO) {
        strcpy(modulestr, "GPIO");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_HWMGR) {
        strcpy(modulestr, "HW Manager");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_I2C) {
        strcpy(modulestr, "I2C");
        if (code == CYHAL_I2C_RSLT_ERR_BAD_ARGUMENT) {
            strcpy(codestr, "Bad argument");
        } else if (code == CYHAL_I2C_RSLT_ERR_ABORT_ASYNC_TIMEOUT) {
            strcpy(codestr, "Async timeout");
        } else if (code == CYHAL_I2C_RSLT_ERR_BUFFERS_NULL_PTR) {
            strcpy(codestr, "RX or TX buffer is not initialized");
        } else if (code == CYHAL_I2C_RSLT_ERR_CAN_NOT_REACH_DR) {
            strcpy(codestr, "Cannot reach the desired data rate");
        } else if (code == CYHAL_I2C_RSLT_ERR_CMD_ERROR) {
            strcpy(codestr, "Command error");
        } else if (code == CYHAL_I2C_RSLT_ERR_INVALID_ADDRESS_SIZE) {
            strcpy(codestr, "Address size is invalid");
        } else if (code == CYHAL_I2C_RSLT_ERR_INVALID_PIN) {
            strcpy(codestr, "Pin is invalid");
        } else if (code == CYHAL_I2C_RSLT_ERR_NO_ACK) {
            strcpy(codestr, "No acknowledge recieved");
        } else if (code == CYHAL_I2C_RSLT_ERR_PM_CALLBACK) {
            strcpy(codestr, "Failed to register PM callback");
        } else if (code == CYHAL_I2C_RSLT_ERR_PREVIOUS_ASYNCH_PENDING) {
            strcpy(codestr, "Previous async operation pending");
        } else if (code == CYHAL_I2C_RSLT_ERR_TX_RX_BUFFERS_ARE_EMPTY) {
            strcpy(codestr, "Transmit or recieve buffer is empty");
        } else if (code == CYHAL_I2C_RSLT_ERR_UNSUPPORTED) {
            strcpy(codestr, "Operation is not supported by this device");
        } else {
            strcpy(codestr, "Unknown error");
        }
    } else if (module == CYHAL_RSLT_MODULE_I2S) {
        strcpy(modulestr, "I2S");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_IMPL_SCB) {
        strcpy(modulestr, "SCB");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_IMPL_TCPWM) {
        strcpy(modulestr, "TCPWM");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_INTERCONNECT) {
        strcpy(modulestr, "Interconnect");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_IPC) {
        strcpy(modulestr, "IPC");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_KEYSCAN) {
        strcpy(modulestr, "Keyscan");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_LPTIMER) {
        strcpy(modulestr, "LPTimer");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_OPAMP) {
        strcpy(modulestr, "Op-amp");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_PDMPCM) {
        strcpy(modulestr, "PWM/PCM");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_PWM) {
        strcpy(modulestr, "PWM");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_QSPI) {
        strcpy(modulestr, "QSPI");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_QUADDEC) {
        strcpy(modulestr, "Quad. decoder");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_RTC) {
        strcpy(modulestr, "RTC");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_SDHC) {
        strcpy(modulestr, "SDHC");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_SDIO) {
        strcpy(modulestr, "SDIO");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_SPI) {
        strcpy(modulestr, "SPI");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_SYSPM) {
        strcpy(modulestr, "SysPM");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_SYSTEM) {
        strcpy(modulestr, "System");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_T2TIMER) {
        strcpy(modulestr, "T2Timer");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_TDM) {
        strcpy(modulestr, "TDM");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_TIMER) {
        strcpy(modulestr, "Timer");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_TRNG) {
        strcpy(modulestr, "RNG");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_UART) {
        strcpy(modulestr, "UART");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_USB) {
        strcpy(modulestr, "USB");
        strcpy(codestr, "Unknown error");
    } else if (module == CYHAL_RSLT_MODULE_WDT) {
        strcpy(modulestr, "WDT");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_BLOCK_STORAGE) {
        strcpy(modulestr, "Block store");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_BSP) {
        strcpy(modulestr, "Board support");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_DATA_STREAMING) {
        strcpy(modulestr, "Data streaming");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_FS) {
        strcpy(modulestr, "File system");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_HAL) {
        strcpy(modulestr, "HAL");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_OS) {
        strcpy(modulestr, "RTOS");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_ABSTRACTION_RESOURCE) {
        strcpy(modulestr, "Resource abs.");
        strcpy(codestr, "Unknown error");
    } else if (module == CY_RSLT_MODULE_DRIVER_SCB) {
        strcpy(modulestr, "SCB");
        if (result == CY_SCB_I2C_BAD_PARAM) {
            strcpy(codestr, "Bad parameters");
        } else if (result == CY_SCB_I2C_MASTER_NOT_READY) {
            strcpy(codestr, "Monarch not ready");
        } else if (result == CY_SCB_I2C_MASTER_MANUAL_TIMEOUT) {
            strcpy(codestr, "Monarch manual timeout");
        } else if (result == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK) {
            strcpy(codestr, "Serf nack'd address");
        } else if (result == CY_SCB_I2C_MASTER_MANUAL_NAK) {
            strcpy(codestr, "Serf nack'd data");
        } else if (result == CY_SCB_I2C_MASTER_MANUAL_ARB_LOST) {
            strcpy(codestr, "Monarch lost arbitration");
        } else if (result == CY_SCB_I2C_MASTER_MANUAL_BUS_ERR) {
            strcpy(codestr, "Monarch bus error");
        } else if (result == CY_SCB_I2C_MASTER_MANUAL_ABORT_START) {
            strcpy(codestr, "Monarch transaction aborted");
        } else {
            strcpy(codestr, "Unknown error");
        }
    } else {
        strcpy(modulestr, "Unknown");
        strcpy(codestr, "Unknown error");
    }

    strcat(resultstr, modulestr);
    strcat(resultstr, " ");
    strcat(resultstr, typestr);
    strcat(resultstr, ": ");
    strcat(resultstr, codestr);
    strcat(resultstr, "\n");

    printf(resultstr);
    printf("Raw code: %li Module code: %i Type code: %i Code: %i\n", 
        result, module, type, code);
        
}

void print_ble_result(cy_en_ble_api_result_t result) {
    /** No Error occurred */
    if (result == CY_BLE_SUCCESS) {
        printf("BT Info: Success (check before calling print_ble_result)\r\n");
    }

    /** Flash operation in progress*/
    else if (result == CY_BLE_INFO_FLASH_WRITE_IN_PROGRESS) {
        printf("BT Error: Flash operation in progress\r\n");
    }

    /** At least one of the input parameters is invalid */
    else if (result == CY_BLE_ERROR_INVALID_PARAMETER) {
        printf("BT Error: Parameters incorrect\r\n");
    }

    /** Operation is not permitted */
    else if (result == CY_BLE_ERROR_INVALID_OPERATION) {
        printf("BT Error: Operation not permitted\r\n");
    }

    /** An internal error occurred in the BLE Stack */
    else if (result == CY_BLE_ERROR_MEMORY_ALLOCATION_FAILED) {
        printf("BT Error: Memory allocation failure\r\n");
    }

    /** Insufficient resources to perform requested operation */
    else if (result == CY_BLE_ERROR_INSUFFICIENT_RESOURCES) {
        printf("BT Error: Insufficient resources\r\n");
    }

    /** OOB data not available */
    else if (result == CY_BLE_ERROR_OOB_NOT_AVAILABLE) {
        printf("BT Error: OOB data not available\r\n");
    }

    /** Connection is required to perform requested operation. Connection not
       present */
    else if (result == CY_BLE_ERROR_NO_CONNECTION) {
        printf("BT Error: No connection\r\n");
    }

    /** No device entity to perform requested operation */
    else if (result == CY_BLE_ERROR_NO_DEVICE_ENTITY) {
        printf("BT Error: No device entity to perform operation\r\n");
    }

    /** Device cannot be added to the White List as it has already been added */
    else if (result == CY_BLE_ERROR_DEVICE_ALREADY_EXISTS) {
        printf("BT Error: Device has already been whitelisted\r\n");
    }

    /** Attempted repeat operation is not allowed */
    else if (result == CY_BLE_ERROR_REPEATED_ATTEMPTS) {
        printf("BT Error: Repeat attempts not allowed\r\n");
    }

    /** GAP role is incorrect */
    else if (result == CY_BLE_ERROR_GAP_ROLE) {
        printf("BT Error: Incorrect GAP role\r\n");
    }

    /** Security operation failed */
    else if (result == CY_BLE_ERROR_SEC_FAILED) {
        printf("BT Error: Security operation failed\r\n");
    }

    /** L2CAP PSM encoding is incorrect */
    else if (result == CY_BLE_ERROR_L2CAP_PSM_WRONG_ENCODING) {
        printf("BT Error: L2PCAP PSM encoding is incorrect\r\n");
    }

    /** L2CAP PSM has already been registered */
    else if (result == CY_BLE_ERROR_L2CAP_PSM_ALREADY_REGISTERED) {
        printf("BT Error: L2PCAP PSM has already been registered\r\n");
    }

    /** L2CAP PSM has not been registered */
    else if (result == CY_BLE_ERROR_L2CAP_PSM_NOT_REGISTERED) {
        printf("BT Error: L2PCAP PSM has not been registered\r\n");
    }

    /** L2CAP connection entity not found */
    else if (result == CY_BLE_ERROR_L2CAP_CONNECTION_ENTITY_NOT_FOUND) {
        printf("BT Error: L2PCAP connection entity not found\r\n");
    }

    /** Specified PSM is out of range */
    else if (result == CY_BLE_ERROR_L2CAP_PSM_NOT_IN_RANGE) {
        printf("BT Error: PSM is out of range\r\n");
    }

    /** Unsupported feature or parameter value */
    else if (result == CY_BLE_ERROR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE) {
        printf("BT Error: Unsupported feature or parameter value\r\n");
    }

    /** Write to Flash is not permitted */
    else if (result == CY_BLE_ERROR_FLASH_WRITE_NOT_PERMITED) {
        printf("BT Error: Write to flash not permitted\r\n");
    }

    /** Error in Flash write */
    else if (result == CY_BLE_ERROR_FLASH_WRITE) {
        printf("BT Error: Error in flash wrtie\r\n");
    }

    /** MIC Authentication failure */
    else if (result == CY_BLE_ERROR_MIC_AUTH_FAILED) {
        printf("BT Error: MIC authentification failure\r\n");
    }

    /** Hardware Failure */
    /** Possible reason - BLE ECO failed to start due to one of the below:
     *   - LFCLK (WCO, PILO, ILO) not present
     *   - Radio bus failure 
     */
    else if (result == CY_BLE_ERROR_HARDWARE_FAILURE) {
        printf("BT Error: Hardware failure\r\n");
    }

    /**GATT DB error codes*/
    /** Invalid attribute handle */
    else if (result == CY_BLE_ERROR_GATT_DB_INVALID_ATTR_HANDLE) {
        printf("BT Error: GATT Invalid attribute handle\r\n");
    }

    /* Profile level API_RESULT codes */
    /** Characteristic notifications disabled */
    else if (result == CY_BLE_ERROR_NTF_DISABLED) {
        printf("BT Error: Profile characteristic notifications disabled\r\n");
    }

    /** Characteristic indications disabled */
    else if (result == CY_BLE_ERROR_IND_DISABLED) {
        printf("BT Error: Profile characteristic indications disabled\r\n");
    }

    /** Controller Busy */
    else if (result == CY_BLE_ERROR_CONTROLLER_BUSY) {
        printf("BT Error: Controller is busy\r\n");
    }

    /** The state is not valid for current operation */
    else if (result == CY_BLE_ERROR_INVALID_STATE) {
        printf("BT Error: State is invalid for current operation\r\n");
    }

    else {
        printf("BT Error: Unknown error\r\n");
    }
}

void print_IR_error(VL53LX_Error error) {
    if (error == VL53LX_ERROR_BUFFER_TOO_SMALL) {
        printf("IR Error: Buffer too small\r\n");
        return;
    } else if (error == VL53LX_ERROR_CONTROL_INTERFACE) {
        printf("IR Error: Control interface error\r\n");
        return;
    } else if (error == VL53LX_ERROR_CALIBRATION_WARNING) {
        printf("IR Error: Calibration warning\r\n");
        return;
    } else if (error == VL53LX_ERROR_COMMS_BUFFER_TOO_SMALL) {
        printf("IR Error: Comms buffer too small\r\n");
        return;
    } else if (error == VL53LX_ERROR_DIVISION_BY_ZERO) {
        printf("IR Error: Division by zero\r\n");
        return;
    } else if (error == VL53LX_ERROR_GPH_ID_CHECK_FAIL) {
        printf("IR Error: Stream count check fail\r\n");
        return;
    } else if (error == VL53LX_ERROR_GPH_SYNC_CHECK_FAIL) {
        printf("IR Error: Error during SPAD init\r\n");
        return;
    } else if (error == VL53LX_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED) {
        printf("IR Error: Tried to use a GPIO that is not supported\r\n");
        return;
    } else if (error == VL53LX_ERROR_GPIO_NOT_EXISTING) {
        printf("IR Error: Tried to use a GPIO that does not exist\r\n");
        return;
    } else if (error == VL53LX_ERROR_INVALID_COMMAND) {
        printf("IR Error: Invalid command\r\n");
        return;
    } else if (error == VL53LX_ERROR_INVALID_PARAMS) {
        printf("IR Error: Invalid parameters\r\n");
        return;
    } else if (error == VL53LX_ERROR_MIN_CLIPPED) {
        printf("IR Error: Invalid calibration data in use\r\n");
        return;
    } else if (error == VL53LX_ERROR_MODE_NOT_SUPPORTED) {
        printf("IR Error: Mode not supported\r\n");
        return;
    } else if (error == VL53LX_ERROR_NONE) {
        printf("IR Error: No error (check for VL53LX_ERROR_NONE before calling print_IR_error)\r\n");
        return;
    } else if (error == VL53LX_ERROR_NOT_IMPLEMENTED) {
        printf("IR Error: Not implemented\r\n");
        return;
    } else if (error == VL53LX_ERROR_NOT_SUPPORTED) {
        printf("IR Error: Not supported\r\n");
        return;
    } else if (error == VL53LX_ERROR_OFFSET_CAL_NO_SAMPLE_FAIL) {
        printf("IR Error: Offset calibration number of samples failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL) {
        printf("IR Error: Offset calibration no SPADs enabled failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_PLATFORM_SPECIFIC_START) {
        printf("IR Error: Platform start error\r\n");
        return;
    } else if (error == VL53LX_ERROR_RANGE_ERROR) {
        printf("IR Error: Range error\r\n");
        return;
    } else if (error == VL53LX_ERROR_REF_SPAD_INIT) {
        printf("IR Error: Reference SPAD init failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_STREAM_COUNT_CHECK_FAIL) {
        printf("IR Error: Stream count check failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_TIME_OUT) {
        printf("IR Error: Timed out\r\n");
        return;
    } else if (error == VL53LX_ERROR_TUNING_PARM_KEY_MISMATCH) {
        printf("IR Error: Tuning parameter key mismatch\r\n");
        return;
    } else if (error == VL53LX_ERROR_UNDEFINED) {
        printf("IR Error: Undefined error\r\n");
        return;
    } else if (error == VL53LX_ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL) {
        printf("IR Error: Crosstalk extraction no sample failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL) {
        printf("IR Error: Crosstalk extraction sigma limit failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_ZONE_CAL_NO_SAMPLE_FAIL) {
        printf("IR Error: Zone calibration no sample failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_ZONE_GPH_ID_CHECK_FAIL) {
        printf("IR Error: Zone calibration GPH ID check failure\r\n");
        return;
    } else if (error == VL53LX_ERROR_ZONE_STREAM_COUNT_CHECK_FAIL) {
        printf("IR Error: Zone calibration stream count check failure\r\n");
        return;
    } else {
        printf("IR Error: Unknown error\r\n");
        return;
    }
}