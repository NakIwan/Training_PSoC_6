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
#include "BMP180.h"

cy_stc_scb_i2c_context_t i2cContext;

extern bmp180_obj bmp180;

void scaning_i2c()
{
	for(uint8_t i=1; i<0x7f; i++)
	{
		if( Cy_SCB_I2C_MasterSendStart(i2c_HW, i, CY_SCB_I2C_WRITE_XFER, 10UL, &i2cContext) == CY_SCB_I2C_SUCCESS )
		printf(" found device on address: 0x%x \r\n", i);
		Cy_SCB_I2C_MasterSendStop(i2c_HW, 10UL, &i2cContext);
	}
}

uint32_t bmp_write_bytes(uint8_t addr, uint8_t *data, uint8_t dataLen, uint32_t timeout)
{
	cy_en_scb_i2c_status_t status=0;
	if( Cy_SCB_I2C_MasterSendStart(i2c_HW, (uint32_t)addr, CY_SCB_I2C_WRITE_XFER, timeout, &i2cContext) == CY_SCB_I2C_SUCCESS )
		{
			for(uint8_t i = 0; i<dataLen; i++)
			status |= Cy_SCB_I2C_MasterWriteByte (i2c_HW, data[i], timeout,&i2cContext);
		}
	status = Cy_SCB_I2C_MasterSendStop(i2c_HW, 10UL, &i2cContext);
	return (uint32_t)status;
}

uint32_t bmp_read_bytes(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t dataLen, uint32_t timeout)
{
	cy_en_scb_i2c_status_t status=0;
	if( Cy_SCB_I2C_MasterSendStart(i2c_HW, (uint32_t)addr, CY_SCB_I2C_WRITE_XFER, timeout, &i2cContext) == CY_SCB_I2C_SUCCESS )
		{
		if( CY_SCB_I2C_SUCCESS == Cy_SCB_I2C_MasterWriteByte (i2c_HW, reg, timeout,&i2cContext) )
			{
				Cy_SCB_I2C_MasterSendReStart(i2c_HW, (uint32_t)addr, CY_SCB_I2C_READ_XFER, timeout, &i2cContext);
				for( uint8_t i = 0; i< dataLen-1; i++)
					Cy_SCB_I2C_MasterReadByte (i2c_HW, CY_SCB_I2C_ACK, val++, timeout, &i2cContext);
					Cy_SCB_I2C_MasterReadByte (i2c_HW, CY_SCB_I2C_NAK, val, timeout, &i2cContext);
			}
		}
	status = Cy_SCB_I2C_MasterSendStop(i2c_HW, 10UL, &i2cContext);
	return (uint32_t)status;
}

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
	if (result != CY_RSLT_SUCCESS){
			CY_ASSERT(0);
		}

	/* Enable global interrupts */
	__enable_irq();

	cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
		printf("hello \r\n");

	if( Cy_SCB_I2C_Init(i2c_HW, &i2c_config, &i2cContext) == CY_SCB_I2C_SUCCESS )
		{
			printf("I2C success \r\n");
			Cy_SCB_I2C_Enable(i2c_HW);
		}

	scaning_i2c();

	if( bmp180.init(BMP180_MODE_STANDARD, BMP180_ADDR, bmp_read_bytes, bmp_write_bytes) == BMP180_SUCCESS )
		printf("Device Found !\r\n");

float tp, press;

	for (;;)
	{
		bmp180.readValue(&tp, &press, 100);
		printf("Temp : %0.2f \tPress : %0.2f \r\n", tp, press);
		cyhal_system_delay_ms(500);
	}
}
