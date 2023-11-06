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

#define BAUD_RATE       9600
#define UART_DELAY      10u
#define RX_BUF_SIZE     32
#define TX_BUF_SIZE     32
#define RX_US100 P10_0
#define TX_US100 P10_1

uint8_t      tx_buf[32];
uint8_t      rx_buf[32];

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

int main(void)
{
    cyhal_uart_t uart_obj;
    uint32_t     actualbaud;

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

    /* Enable global interrupts */
    __enable_irq();

	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
	printf("Success\r\n");

   // Initialize the UART Block
    result = cyhal_uart_init(&uart_obj, TX_US100, RX_US100, NC, NC, NULL, &uart_config);


    result = cyhal_uart_set_baud(&uart_obj, BAUD_RATE, &actualbaud);
    //if (result == CY_RSLT_SUCCESS)

    for (;;)
    {

    	if (cyhal_uart_putc(&uart_obj, 0X55) == CY_RSLT_SUCCESS){

    		rx_length = 3;
            cyhal_uart_read(&uart_obj, rx_buf, &rx_length);

//            uint8_t length = sizeof(rx_buf) / sizeof(*rx_buf);
//


            Distance = (uint16_t)rx_buf[0] << 8 | rx_buf[1];
            printf("value = %d\r\n",Distance);
    	}
        cyhal_system_delay_ms(1000);
    }
}

//int sender(uint8_t *arr,int n)
//{
//    uint8_t checksum,sum=0,i;
//
//    printf("\n****SENDER SIDE****\n\r");
//    for(i=0;i<n;i++){
//    	printf("Data arr = %d\r\n",arr[i]);
//    	sum+=arr[i];
//    }
//    printf("SUM IS: %d",sum);
//    checksum=~sum;
//    printf("\n\rCHECKSUM IS:%d\n\r",checksum);
//    return checksum;
//}
//
//void receiver(uint8_t *arr,int n,int sch)
//{
//	uint8_t checksum,sum=0,i;
//    printf("\n\n****RECEIVER SIDE****\n\r");
//    for(i=0;i<n;i++)
//        sum+=arr[i];
//    printf("SUM IS:%d",sum);
//    sum=sum+sch;
//    checksum=~sum;    //1's complement of sum
//    printf("\n\rCHECKSUM IS:%d\r\n",checksum);
//}
