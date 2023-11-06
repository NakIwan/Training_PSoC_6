/*******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Empty Application Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

#define BAUD_RATE 115200

uint8_t rx_data;
uint32_t actualbaud;

uint8_t rx_buf[32];
size_t rx_len = 4;

cyhal_uart_t uart_obj;
const cyhal_uart_cfg_t uart_config =
{
	.data_bits = 8,
	.stop_bits = 1,
	.parity = CYHAL_UART_PARITY_NONE,
	.rx_buffer = rx_buf,
	.rx_buffer_size = 32
};

int main(void)
{

    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cyhal_gpio_init(P6_3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
    result = cyhal_gpio_init(P7_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	//printf("Success\r\n");

	result = cyhal_uart_init(&uart_obj, P10_1, P10_0, NC, NC, NULL, &uart_config);
	if(result == CY_RSLT_SUCCESS)printf("success\r\n");
    result = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actualbaud);

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {
		result = cyhal_uart_getc(&uart_obj, &rx_data, 100);
		if(result == CY_RSLT_SUCCESS){
			printf("Data %c\r\n",(char)rx_data);
			switch((char)rx_data)
			{
			case '0':
				cyhal_gpio_write(P7_1, 0);
				cyhal_gpio_write(P6_3, 1);
				cyhal_system_delay_ms(500);
				break;
			case '1':
				cyhal_gpio_write(P7_1, 1);
				cyhal_gpio_write(P6_3, 0);
				cyhal_system_delay_ms(1000);
				break;
			case '2':
				cyhal_gpio_write(P7_1, 0);
				cyhal_gpio_write(P6_3, 0);
				cyhal_system_delay_ms(500);
				break;
			case '3':
				cyhal_gpio_write(P7_1, 0);
				cyhal_gpio_write(P6_3, 1);
				cyhal_system_delay_ms(100);
				cyhal_gpio_write(P7_1, 1);
				cyhal_gpio_write(P6_3, 0);
				cyhal_system_delay_ms(100);
				break;
			case '4':
				for(int i = 1; i <= 5;i++){
					cyhal_gpio_write(P7_1, 0);
					cyhal_gpio_write(P6_3, 1);
					cyhal_system_delay_ms(100);
					cyhal_gpio_write(P7_1, 1);
					cyhal_gpio_write(P6_3, 0);
					cyhal_system_delay_ms(100);
				}

				break;
			}
		}
		cyhal_gpio_write(P7_1, 1);
		cyhal_gpio_write(P6_3, 1);
    }
}
