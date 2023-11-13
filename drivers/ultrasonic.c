/*
 * push_button.c
 *
 *  Created on: October 20, 2023
 *      Author: Harry Kiffel
 */

#include "ultrasonic.h"

// 
void ultrasonic_init(void){
	
    // Config Trigger pin as output
    cyhal_gpio_init(
			PIN_TRIGGER,                // Pin
			CYHAL_GPIO_DIR_OUTPUT,      // Direction
			CYHAL_GPIO_DRIVE_STRONG,    // Drive Mode
			false);

    // Config Echo1 pin as input
    cyhal_gpio_init(
			PIN_ECHO1,                // Pin
			CYHAL_GPIO_DIR_INPUT,      // Direction
			CYHAL_GPIO_DRIVE_NONE,    // Drive Mode
			false);

    // Config Echo2 pin as input
    cyhal_gpio_init(
			PIN_ECHO2,                // Pin
			CYHAL_GPIO_DIR_INPUT,      // Direction
			CYHAL_GPIO_DRIVE_NONE,    // Drive Mode
			false);

}
void ultrasonic_trigger(void){

    // Start 10us trigger pulse
    cyhal_gpio_write(PIN_TRIGGER, true);
    printf("Trigger Start\r\n");
    // Delay 10us
    cyhal_system_delay_ms(10);	
    printf("Trigger Complete\r\n");
    // Stop Pulse
    cyhal_gpio_write(PIN_TRIGGER, false);
}

// Basic pulse interpreter for Echo pins
uint32_t ultrasonic_receive(cyhal_gpio_t echopin){
    printf("Start Receive\r\n");
    cy_rslt_t rslt;
    uint32_t read_val;
    /* Timer object used */
    cyhal_timer_t timer_obj;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = 200000,                    /* Timer period set to a large enough value
                                             * compared to event being measured */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = false,             /* Do not run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };
    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&timer_obj, NC, NULL);
    printf("Timer Initialized\r\n");
    /* Apply timer configuration such as period, count direction, run mode, etc. */
    cyhal_timer_configure(&timer_obj, &timer_cfg);
    /* Set the frequency of timer to 10000 counts in a second or 10000 Hz */
    cyhal_timer_set_frequency(&timer_obj, 1000000);
    printf("Pre-trigger\r\n");
    // trigger sensor signal
    ultrasonic_trigger();

    // wait until echo pulse starts
    while(cyhal_gpio_read(echopin) == 0);
    /* Start the timer with the configured settings */
    cyhal_timer_start(&timer_obj);
    printf("Timer Start");
    /* Read Echo PIN for High */ 
    while(cyhal_gpio_read(echopin) == 1);
    /* Read the current timer value, which should be close to the amount of delay in ms * 10 (5000) */
    read_val = cyhal_timer_read(&timer_obj);
    return read_val;
}

// Basic Distance capture for specified echo pin.
float ultrasonic_get_object_distance(cyhal_gpio_t echopin){
    uint32_t time_us = ultrasonic_receive(echopin);
    float cm_distance = 0.0343 * time_us;
    return cm_distance;
}