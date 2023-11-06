/******************************************************************************
* File Name: main.c
*
* Description: This example demonstrates the UART transmit and receive
*              operation using HAL APIs
*
* Related Document: See Readme.md
*
*******************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define BAUD_RATE       9600
#define UART_DELAY      10u
#define RX_BUF_SIZE     32
#define TX_BUF_SIZE     32
#define RX_US100 P10_0
#define TX_US100 P10_1


uint8_t      tx_buf[TX_BUF_SIZE];
uint8_t      rx_buf[RX_BUF_SIZE];
uint8_t      read_data[RX_BUF_SIZE];

uint16_t Distance = 0X55;
uint16_t sendTemperature = 0X50;

size_t       tx_length = TX_BUF_SIZE;
size_t       rx_length = RX_BUF_SIZE;

const cyhal_uart_cfg_t uart_config =
{
	.data_bits = 8,
	.stop_bits = 1,
	.parity = CYHAL_UART_PARITY_NONE,
	.rx_buffer = rx_buf,
	.rx_buffer_size = RX_BUF_SIZE
};

uint32_t buffer[32];
uint16_t trigger;

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
* Reads one byte from the serial terminal and echoes back the read byte.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cyhal_uart_t uart_obj;
    uint32_t     actualbaud;

    cy_rslt_t result;
    #if defined(CY_DEVICE_SECURE)
        cyhal_wdt_t wdt_obj;
        /* Clear watchdog timer so that it doesn't trigger a reset */
        result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
        CY_ASSERT(CY_RSLT_SUCCESS == result);
        cyhal_wdt_free(&wdt_obj);
    #endif

 /* Variable to store the received character
                        * through terminal */

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX,
                                    CYBSP_DEBUG_UART_RX,
                                    CYBSP_DEBUG_UART_CTS,
                                    CYBSP_DEBUG_UART_RTS,
                                    CY_RETARGET_IO_BAUDRATE);

    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    __enable_irq();

    // Initialize the UART Block
     result = cyhal_uart_init(&uart_obj, TX_US100, RX_US100, NC, NC, NULL, &uart_config);


     result = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actualbaud);
     //if (result == CY_RSLT_SUCCESS)

    for (;;)
    {

        	if (cyhal_uart_putc(&uart_obj, 0X55) == CY_RSLT_SUCCESS){

        		rx_length = 3;
                cyhal_uart_read(&uart_obj, rx_buf, &rx_length);
                Distance = (uint16_t)rx_buf[0] << 8 | rx_buf[1];
                printf("value = %d\r\n",Distance/10);

        	}
            cyhal_system_delay_ms(1000);
    }
}
