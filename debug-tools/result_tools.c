#include "result_tools.h"
#include "../drivers/console.h"
#include "cyhal_general_types.h"

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
        strcpy(codestr, "Unknown error");
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