/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the CYW20829 Clock Frequency
* Measurement Example for ModusToolbox. This example demonstrates how to use a
* clock measurement counter to measure the clock frequency using another clock
* as reference.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cy_sysclk.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define LED_DELAY_MS            (1000)     /* milliseconds */
#define CLOCK1_INIT_VAL         (0x3FFUL)  /*1024*/


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static uint32_t get_clk_measurements(uint32_t clk);
void print_all_clocks(void);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. Initialize the user LED and toggle the LED at one second interval.
*   2. Calls the "print_all_clocks" to print the frequency of configured clocks.
*
*Parameters: void
*
* Return: int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                 CY_RETARGET_IO_BAUDRATE);

    /* Retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("*************************************** "
           "Clock Frequency Measurement Application "
           "*************************************** \r\n\n");

    /* Initialize the user LED */
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* led init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    print_all_clocks();

    for (;;)
    {
        /* Toggle user LED for every one second */
        cyhal_gpio_toggle(CYBSP_USER_LED);
        cyhal_system_delay_ms(LED_DELAY_MS);
    }
}

/*******************************************************************************
* Function Name: print_all_clocks
********************************************************************************
* Summary:
*  Measures the frequency of the clock and prints the frequency on terminal.
*
* Parameters:
*  void
*
* Return: void
*
*******************************************************************************/
void print_all_clocks(void)
{
    printf("Clock[0x%x] = %lu\r\n",CY_SYSCLK_MEAS_CLK_CLKHF0,
            (unsigned long)get_clk_measurements(CY_SYSCLK_MEAS_CLK_CLKHF0));
    printf("Clock[0x%x] = %lu\r\n",CY_SYSCLK_MEAS_CLK_CLKHF1,
            (unsigned long)get_clk_measurements(CY_SYSCLK_MEAS_CLK_CLKHF1));
    printf("Clock[0x%x] = %lu\r\n",CY_SYSCLK_MEAS_CLK_CLKHF2,
            (unsigned long)get_clk_measurements(CY_SYSCLK_MEAS_CLK_CLKHF2));
    printf("Clock[0x%x] = %lu\r\n",CY_SYSCLK_MEAS_CLK_CLKHF3,
            (unsigned long)get_clk_measurements(CY_SYSCLK_MEAS_CLK_CLKHF3));

    printf("After CLK_DIRECT Select as IMO\r\n");
    Cy_SysClk_ClkHfDirectSel(0, true);
    printf("Clock[0x%x] = %lu\r\n",CY_SYSCLK_MEAS_CLK_CLKHF0,
            (unsigned long)get_clk_measurements(CY_SYSCLK_MEAS_CLK_CLKHF0));
    printf("After CLK_DIRECT Select as ROOT_MUX\r\n");
    Cy_SysClk_ClkHfDirectSel(0, false);
    printf("Clock[0x%x] = %lu\r\n",CY_SYSCLK_MEAS_CLK_CLKHF0,
            (unsigned long)get_clk_measurements(CY_SYSCLK_MEAS_CLK_CLKHF0));
    printf("FLL Clock = %lu\r\n", (unsigned long)Cy_SysClk_FllGetFrequency());
    printf("ALTHF Enabled status = %lu\r\n",
            (unsigned long)Cy_SysClk_IsAltHfEnabled());
}

/*******************************************************************************
* Function Name: get_clk_measurements
********************************************************************************
* Summary:
*  Calculates the frequency of input clock.
*
* Parameters:
*  clk: clk whose frequency need to be measured.
*
* Return: Cy_SysClk_ClkMeasurementCountersGetFreq: Returns the frequency of the
*  input clock.
*
*******************************************************************************/
static uint32_t get_clk_measurements(uint32_t clk)
{
    /* Start the clock measurement using the IMO as reference */
    /* Counter 1 clock init value = 1024 and counter 2 clock = IMO */
    (void)Cy_SysClk_StartClkMeasurementCounters((cy_en_meas_clks_t)clk,
            CLOCK1_INIT_VAL, CY_SYSCLK_MEAS_CLK_IMO);

    /* Wait for counter 1 to reach 0 */
    while (!Cy_SysClk_ClkMeasurementCountersDone()){};

    /* Measure clock 1 with the IMO clock cycles (counter 2) */
    return Cy_SysClk_ClkMeasurementCountersGetFreq(false, CY_SYSCLK_IMO_FREQ);
}

/* [] END OF FILE */
