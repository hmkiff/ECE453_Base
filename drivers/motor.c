/*
 * push_button.c
 *
 *  Created on: October 9, 2023
 *      Author: Harry Kiffel
 */

#include "motor.h"


/* IO Pin Handler Struct */
cyhal_gpio_callback_data_t motor_cb_data =
{
    .callback     = motor_handler,
    .callback_arg = (void*)NULL
};

/***********************************************************
* Handler for SW2
************************************************************/
void motor_handler(void* handler_arg, cyhal_gpio_event_t event)
{

    /*ADD CODE to toggle the LED ON/Off) */
	cyhal_gpio_toggle(PIN_USER_LED);
}

/***********************************************************
* Initialize SW2 Interrupts
************************************************************/
static void motor_irq_init(void)
{
	// Enable the interrupt for the CapSense Change IRQ
    cyhal_gpio_register_callback(
    		PIN_MOTOR, 		    // Pin
			&motor_cb_data);		// Handler Callback Info

    cyhal_gpio_enable_event(
			PIN_MOTOR,	       	// Pin
			CYHAL_GPIO_IRQ_FALL, 	    // Event
			INT_PRIORITY_MOTOR,   // Priority
			true);			            // Enable Event
}

/*****************************************************
* Function Name: motor_io_init
******************************************************
* Summary:
*
* Parameters:
*  void
*
* Return:
*
*
*****************************************************/
static void motor_io_init(void)
{
    /* ADD CODE to configure motor as an output */
	cyhal_gpio_init(
			PIN_MOTOR,            // Pin
			CYHAL_GPIO_DIR_INPUT,      // Direction
			CYHAL_GPIO_DRIVE_NONE,     // Drive Mode
			false);
}


/*****************************************************
* Function Name: motor_init
******************************************************
* Summary:
* Initializes the IO pin used to control the motor
*
* Parameters:
*  void
*
* Return:
*
*
*****************************************************/
void motor_init(void)
{
	motor_io_init();
	motor_irq_init();
}
