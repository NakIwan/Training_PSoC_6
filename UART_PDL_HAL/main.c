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
#include "cy_pdl.h"
#include "cy_retarget_io.h"
/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CPU. It...
*    1.
*    2.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/

volatile uint32 SysTickCount;
void SysTickCallback(void)
{
SysTickCount++;
}

uint32_t getTickCount(){return SysTickCount;}

cyhal_uart_t uart_obj;
const cyhal_uart_cfg_t uart_config =
{
	.data_bits = 8,
	.stop_bits = 1,
	.parity = CYHAL_UART_PARITY_NONE,
	.rx_buffer = NULL,
	.rx_buffer_size = 0
};

int main(void)
{
    cy_rslt_t result;
    uint8_t read_data;

	//cy_stc_scb_uart_context_t UART_context;

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

    result = cyhal_gpio_init(P7_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);//Initialization pin on HAL
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/*result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);//Initialization UART on HAL
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, (SystemCoreClock/1000)-1);
	Cy_SysTick_SetCallback(0,SysTickCallback);
	Cy_SysTick_Enable();*/

	result = cyhal_uart_init(&uart_obj, CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, NC, NC, NULL, &uart_config);
	if (result != CY_RSLT_SUCCESS)
		{
			CY_ASSERT(0);
		}

    __enable_irq();



	/*Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
	Cy_SCB_UART_Enable(UART_HW);*/

    for (;;)
    {

    	cyhal_uart_getc(&uart_obj, &read_data, 0);
    	if (read_data == 0){
    		cyhal_gpio_write(P7_1, 0);
    	}
    	if (read_data == 1){
    		cyhal_gpio_write(P7_1, 1);
    	}

    	/*cyhal_gpio_write(P0_4, CYBSP_LED_STATE_ON);
    	printf("LED ON\n\r");
    	cyhal_system_delay_ms(250);
    	cyhal_gpio_write(P7_1, CYBSP_LED_STATE_OFF);
    	printf("LED OFF\n\r");
    	cyhal_system_delay_ms(250);

    	Cy_GPIO_Set(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM);
    	Cy_SCB_UART_PutString(UART_HW, "LED OFF\n");
    	Cy_SysLib_Delay(250U);
    	Cy_GPIO_Clr(CYBSP_USER_LED1_PORT, CYBSP_USER_LED1_NUM);
    	Cy_SCB_UART_PutString(UART_HW, "LED ON\n");
		Cy_SysLib_Delay(250U);*/
    }
}

/* [] END OF FILE */
