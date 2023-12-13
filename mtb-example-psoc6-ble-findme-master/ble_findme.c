/******************************************************************************
* File Name: ble_findme.c
*
* Description: This file contains BLE related functions.
*
* Related Document: README.md
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/


/******************************************************************************
 * Include header files
 *****************************************************************************/
#include "ble_findme.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define BLESS_INTR_PRIORITY       (1u)
#define WAKEUP_INTR_PRIORITY      (7u)
#define WAKEUP_TIMER_DELAY_MS     (250)
/* Timer value to get 0.25 sec with wakeup timer input clock of 32768 Hz,
 * where 32768Hz is LFCLK in PSoC 6 MCU */
#define WAKEUP_TIMER_MATCH_VALUE  (WAKEUP_TIMER_DELAY_MS * 32768 / 1000)


/*******************************************************************************
* Global Variables
********************************************************************************/
cyhal_lptimer_t wakeup_timer;
bool wakeup_intr_flag = false;
bool gpio_intr_flag = false;
uint8 alert_level = CY_BLE_NO_ALERT;
cy_stc_ble_conn_handle_t app_conn_handle;

/* ECE453 EDIT */
cy_stc_ble_gatt_write_param_t *write_req_param;
cy_stc_ble_gatts_char_val_read_req_t *read_req_param;

/* Team 06 edit */
volatile bool ALERT_BT_RX = 0;
char btInputString[BT_MESSAGE_MAX_LEN];

#ifndef NUM_BOTS
#define NUM_BOTS 4
#endif 

uint8_t bot_0_addr[CY_BLE_BD_ADDR_SIZE] = {157, 16, 38, 80, 160, 0};
uint8_t bot_1_addr[CY_BLE_BD_ADDR_SIZE] = {156, 19, 38, 80, 160, 0};
uint8_t bot_2_addr[CY_BLE_BD_ADDR_SIZE] = {59, 24, 38, 80, 160, 0};
uint8_t bot_3_addr[CY_BLE_BD_ADDR_SIZE] = {181, 16, 38, 80, 160, 0};
uint8_t* addr_ptrs[NUM_BOTS] = {bot_0_addr, bot_1_addr, bot_2_addr, bot_3_addr};

uint8_t conn_bot_0_handle = 0;
uint8_t conn_bot_1_handle = 0;
uint8_t conn_bot_2_handle = 0;
uint8_t conn_bot_3_handle = 0;
volatile uint8_t* conn_bot_ptrs[NUM_BOTS] = {&conn_bot_0_handle, &conn_bot_1_handle, &conn_bot_2_handle, &conn_bot_3_handle};

uint8_t botstate_attrs[NUM_BOTS] = {
    CY_BLE_BOTSTATES_BOTSTATE_0_CHAR_HANDLE, 
    CY_BLE_BOTSTATES_BOTSTATE_1_CHAR_HANDLE,
    CY_BLE_BOTSTATES_BOTSTATE_2_CHAR_HANDLE,
    CY_BLE_BOTSTATES_BOTSTATE_3_CHAR_HANDLE
};

volatile int i_am = -1;
volatile int server_is = -1;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void ble_init(void);
static void bless_interrupt_handler(void);
static void stack_event_handler(uint32 event, void* eventParam);
static void ble_start_advertisement(void);
static void ble_ias_callback(uint32 event, void *eventParam);
static void ble_chain_join_connect(uint8_t* addr);
static int addr_is_bot(uint8_t* addr);


/*******************************************************************************
* Function Name: ble_findme_init
********************************************************************************
* Summary:
* This function initializes the BLE and a deep sleep wakeup timer.
*
*******************************************************************************/
void ble_findme_init(void)
{
    /* Configure BLE */
    ble_init();

    /* Enable global interrupts */
    __enable_irq();
}

/*******************************************************************************
* Function Name: ble_findme_process
********************************************************************************
* Summary:
*  This function processes the BLE events and configures the device to enter
*  low power mode as required.
*
*******************************************************************************/
void ble_findme_process(void) {

    /* Cy_BLE_ProcessEvents() allows the BLE stack to process pending events */
    Cy_BLE_ProcessEvents();

    if(wakeup_intr_flag)
    {
        wakeup_intr_flag = false;

        /* Update ECE453_USR_LED to indicate current BLE status */
        if(CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState())
        {
            cyhal_gpio_toggle((cyhal_gpio_t)ECE453_USR_LED);
        }
        else if(CY_BLE_CONN_STATE_CONNECTED == Cy_BLE_GetConnectionState(app_conn_handle))
        {
            cyhal_gpio_write((cyhal_gpio_t)ECE453_USR_LED, CYBSP_LED_STATE_ON);
        }
        else
        {
            cyhal_gpio_write((cyhal_gpio_t)ECE453_USR_LED, CYBSP_LED_STATE_OFF);
        }
    }
}


/*******************************************************************************
* Function Name: ble_init
********************************************************************************
* Summary:
*  This function initializes the BLE and registers IAS callback function.
*
*******************************************************************************/
static void ble_init(void)
{
    static const cy_stc_sysint_t bless_isr_config =
    {
      /* The BLESS interrupt */
      .intrSrc = bless_interrupt_IRQn,

      /* The interrupt priority number */
      .intrPriority = BLESS_INTR_PRIORITY
    };

    /* Hook interrupt service routines for BLESS */
    (void) Cy_SysInt_Init(&bless_isr_config, bless_interrupt_handler);

    /* Store the pointer to blessIsrCfg in the BLE configuration structure */
    cy_ble_config.hw->blessIsrConfig = &bless_isr_config;

    /* Registers the generic callback functions  */
    Cy_BLE_RegisterEventCallback(stack_event_handler);

    /* Initializes the BLE host */
    Cy_BLE_Init(&cy_ble_config);

    /* Enables BLE */
    Cy_BLE_Enable();

    /* Enables BLE Low-power mode (LPM)*/
    Cy_BLE_EnableLowPowerMode();

    /* Register IAS event handler */
    Cy_BLE_IAS_RegisterAttrCallback(ble_ias_callback);
}


/******************************************************************************
* Function Name: bless_interrupt_handler
*******************************************************************************
* Summary:
*  Wrapper function for handling interrupts from BLESS.
*
******************************************************************************/
static void bless_interrupt_handler(void)
{
    Cy_BLE_BlessIsrHandler();
}


/*******************************************************************************
* Function Name: stack_event_handler
********************************************************************************
*
* Summary:
*   This is an event callback function to receive events from the BLE Component.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParam:  parameters related to the event
*
*******************************************************************************/
static void stack_event_handler(uint32_t event, void* eventParam)
{
    switch(event)
    {
        /**********************************************************************
         * General events
         *********************************************************************/

        /* This event is received when the BLE stack is started */
        case CY_BLE_EVT_STACK_ON:
        {
            printf("[INFO] : BLE stack started \r\n");
            ble_start_advertisement();
            break;
        }

        /* This event is received when there is a timeout */
        case CY_BLE_EVT_TIMEOUT: {

            /* Reason for Timeout */
            cy_en_ble_to_reason_code_t reason_code =
                ((cy_stc_ble_timeout_param_t*)eventParam)->reasonCode;

            switch (reason_code) {
                case CY_BLE_GAP_ADV_TO: {
                    printf("[INFO] : Advertisement timeout event \r\n");
                    break;
                } case CY_BLE_GAP_SCAN_TO: {
                    printf("[INFO] : Scan timeout event \r\n");
                    break;
                } case CY_BLE_GATT_RSP_TO: {
                    printf("[INFO] : GATT response timeout\r\n");
                    break;
                } default: {
                    printf("[INFO] : BLE timeout event\r\n");
                    break;
                }
            }
            break;
        }

        /**********************************************************************
         * GAP events
         *********************************************************************/
        
        case CY_BLE_EVT_GAPC_SCAN_START_STOP: {
            // TODO Might be nice to know which
            printf("BT Chain Info: Scan start or stop occurred\r\n");
        }

        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT: {
            cy_stc_ble_gapc_adv_report_param_t* param = 
                (cy_stc_ble_gapc_adv_report_param_t*) eventParam;

            // Is it a bot?
            if (addr_is_bot(param->peerBdAddr) != -1) {
                
                // Report the found bot
                server_is = addr_is_bot(param->peerBdAddr);
                printf("BT Chain Info: Found server bot %i at ", server_is);
                for (int j = 0; j < CY_BLE_BD_ADDR_SIZE; j++) {
                    printf("%i", param->peerBdAddr[j]);
                    if (j < CY_BLE_BD_ADDR_SIZE) {
                        printf(":");
                    }
                }
                printf(", joining...\r\n");

                // Connect
                ble_chain_join_connect(param->peerBdAddr);
            }
        }

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device
         */
        case CY_BLE_EVT_GAP_DEVICE_CONNECTED: {
            cy_stc_ble_gap_connected_param_t* param = 
                (cy_stc_ble_gap_connected_param_t*) eventParam;
            
            int bot_ind = addr_is_bot(param->peerAddr);
            bool is_bot = bot_ind != -1;
            if (is_bot && (param->status == 0)) {
                printf("BT Chain Info: Bot GAP device connected \r\n");
                conn_bot_ptrs[bot_ind] = param->bdHandle;
            } else if (is_bot){
                printf("BT Chain Error: Bot GAP connection failed with status %i\r\n", param->status);
            }
            break;
        }

        /* This event is generated when disconnected from remote device or
         * failed to establish connection
         */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED: {
            cy_stc_ble_gap_disconnect_param_t* param = 
                (cy_stc_ble_gap_disconnect_param_t*) eventParam;

            if (CY_BLE_CONN_STATE_DISCONNECTED ==
                Cy_BLE_GetConnectionState(app_conn_handle)) {
                for (int i = 0; i < (NUM_BOTS - 1); i++) {
                    if (*(conn_bot_ptrs[i]) == param->bdHandle) {
                        printf("BT Chain Info : Bot GAP device disconnected \r\n");
                        conn_bot_ptrs[i] = 0;
                    }
                }
                ble_start_advertisement();
            }
            break;
        }

        case CY_BLE_EVT_GAP_CREATE_CONN_CANCEL_COMPLETE: {
            printf("[INFO] : GAP Connection canceled, retrying...\r\n");
            ble_chain_join();
        }

        /* This event indicates that the GAP Peripheral device has
         * started/stopped advertising
         */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP: {
            if (CY_BLE_ADV_STATE_ADVERTISING == Cy_BLE_GetAdvertisementState()) {
                printf("[INFO] : BLE advertisement started\r\n");
            } else {
                printf("[INFO] : BLE advertisement stopped\r\n");
            }
            break;
        }


        /**********************************************************************
         * GATT events
         *********************************************************************/

        /* This event is generated at the GAP Peripheral end after connection
         * is completed with peer Central device
         */
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            app_conn_handle = *(cy_stc_ble_conn_handle_t *)eventParam;
            printf("[INFO] : GATT device connected\r\n");
            break;
        }

        /* This event is generated at the GAP Peripheral end after disconnection */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
        {
            printf("[INFO] : GATT device disconnected\r\n");
            break;
        }

        /* This event indicates that the 'GATT MTU Exchange Request' is received */
        case CY_BLE_EVT_GATTS_XCNHG_MTU_REQ:
        {
            printf("[INFO] : GATT MTU Exchange Request received \r\n");
            break;
        }

        /* This event received when GATT read characteristic request received */
        /* ECE453 Read Characteristic START*/
        case CY_BLE_EVT_GATTS_READ_CHAR_VAL_ACCESS_REQ:
        {
            printf("[INFO] : GATT read characteristic request received \r\n");
            read_req_param = (cy_stc_ble_gatts_char_val_read_req_t *)eventParam;

            /*
            if (CY_BLE_BUTTONS_USR_BTN_CHAR_HANDLE == read_req_param->attrHandle) {
            	CY_BLE_GATT_DB_ATTR_SET_GEN_VALUE(CY_BLE_BUTTONS_USR_BTN_CHAR_HANDLE,&BTN_COUNT,1);

                printf("[INFO] : BTN_COUNT %d\r\n", BTN_COUNT);
            }
            */

            break;
        }
        /* ECE453 Read Characteristic END*/

        /* ECE453 Write Characteristic START*/
        case CY_BLE_EVT_GATTS_WRITE_CMD_REQ: {
            printf("[INFO] : GATT write characteristic request received \r\n");
			write_req_param = (cy_stc_ble_gatt_write_param_t*)eventParam;

            cy_ble_gatt_db_attr_handle_t handle = write_req_param->handleValPair.attrHandle;
            if (handle == CY_BLE_CMD_USR_CMD_CHAR_HANDLE) {
                strcpy(btInputString, write_req_param->handleValPair.value.val);
                for (int i = 0; i < BT_MESSAGE_MAX_LEN; i++) {
                    write_req_param->handleValPair.value.val[i] = '\0';
                }
                printf("[INFO] : BT Command recieved: ");
                printf(btInputString);
                printf("\r\n");

                ALERT_BT_RX = true;
            } else if (handle == CY_BLE_BOTSTATES_BOTSTATE_0_CHAR_HANDLE) {
                printf("[INFO] : Write to botstate 0 requested \r\n");
            } else if (handle == CY_BLE_BOTSTATES_BOTSTATE_1_CHAR_HANDLE) {
                printf("[INFO] : Write to botstate 1 requested \r\n");
            } else if (handle == CY_BLE_BOTSTATES_BOTSTATE_2_CHAR_HANDLE) {
                printf("[INFO] : Write to botstate 2 requested \r\n");
            } else if (handle == CY_BLE_BOTSTATES_BOTSTATE_3_CHAR_HANDLE) {
                printf("[INFO] : Write to botstate 3 requested \r\n");
            }
			break;
		} 
        default: {
            printf("[INFO] : BLE Event 0x%lX\r\n", (unsigned long) event);
        }
    }
}


/*******************************************************************************
* Function Name: ble_ias_callback
********************************************************************************
* Summary:
*  This is an event callback function to receive events from the BLE, which are
*  specific to Immediate Alert Service.
*
* Parameters:
*  uint32 event:      event from the BLE component
*  void* eventParams: parameters related to the event
*
*******************************************************************************/
void ble_ias_callback(uint32 event, void *eventParam)
{
    /* Alert Level Characteristic write event */
    if(event == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL,
                                           sizeof(alert_level), &alert_level);
    }

    /* Remove warning for unused parameter */
    (void)eventParam;
}


/******************************************************************************
* Function Name: ble_start_advertisement
*******************************************************************************
* Summary:
*  This function starts the advertisement.
*
******************************************************************************/
static void ble_start_advertisement(void)
{
    cy_en_ble_api_result_t ble_api_result;

    if((CY_BLE_ADV_STATE_ADVERTISING != Cy_BLE_GetAdvertisementState()) &&
       (Cy_BLE_GetNumOfActiveConn() < CY_BLE_CONN_COUNT))
    {
        ble_api_result = Cy_BLE_GAPP_StartAdvertisement(
                            CY_BLE_ADVERTISING_FAST,
                            CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);

        if(CY_BLE_SUCCESS != ble_api_result)
        {
            printf("[ERROR] : Failed to start advertisement \r\n");
        }
    }
}

// Bot chain -----------------------------------------------------------------

void ble_chain_start() {
    i_am = addr_is_bot(cy_ble_configPtr->deviceAddress->bdAddr);
    server_is = i_am;

    printf("BT Chain Info: Address is ");
    for (int i = 0; i < CY_BLE_BD_ADDR_SIZE; i++) {
        printf("%i:", cy_ble_configPtr->deviceAddress->bdAddr[i]);
    }
    printf(", I am %i\r\n", i_am);
}

void ble_chain_join() {
    i_am = addr_is_bot(cy_ble_configPtr->deviceAddress->bdAddr);

    // Start scanning for a match
    cy_en_ble_api_result_t result = Cy_BLE_GAPC_StartScan (
        CY_BLE_SCANNING_FAST,
        0
    );
    if (result != CY_BLE_SUCCESS) {
        printf("BT Error: Couldn't start scan, reason:\r\n");
        print_ble_result(result);
        return;
    }
}

void ble_chain_join_connect(uint8_t* addr) {
    cy_en_ble_api_result_t result = Cy_BLE_GAPC_StopScan();
    if (result != CY_BLE_SUCCESS) {
        printf("BT Chain Error: Couldn't stop scan, reason:\r\n");
        print_ble_result(result);
        return;
    }

    cy_stc_ble_bd_addr_t address;
    for (int i = 0; i < CY_BLE_BD_ADDR_SIZE; i++) {
        address.bdAddr[i] = addr[i];
    }
    address.type = 0;
    result = Cy_BLE_GAPC_ConnectDevice(
        &address,
        0
    );
    if (result != CY_BLE_SUCCESS) {
        printf("BT Chain Error: Couldn't connect to bot, reason:\r\n");
        print_ble_result(result);
        return;
    }
}

// Ret -1 if no, self index if yes
int addr_is_bot(uint8_t* addr) {
    for (int i = 0; i < NUM_BOTS; i++) {
        bool found = true;
        uint8_t* this_addr = addr_ptrs[i];
        for (int j = 0; j < CY_BLE_BD_ADDR_SIZE; j++) {
            if (addr[j] != this_addr[j]) {
                found = false;
                break;
            }
        }
        if (found) {
            return i;
        }
    }
    return -1;
}