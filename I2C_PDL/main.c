
/*******************************************************************************
* Program read address for I2C
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"

cy_stc_scb_i2c_context_t i2cContext;

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

    /* Enable global interrupts */
    __enable_irq();

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = Cy_SCB_I2C_Init(i2c_HW, &i2c_config, &i2cContext);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_I2C_Enable(i2c_HW);

    for (;;)
    {
    	for (uint8_t index = 1; index < 0X7f ; index++)
    	{
    		//printf("index = %X \r\n", index);
    		uint32_t test = CY_SCB_I2C_ID;
    		result = Cy_SCB_I2C_MasterSendStart(i2c_HW, index, 0, 100UL, &i2cContext);
    		if ( result == CY_SCB_I2C_SUCCESS )
    			printf(" found address at 0x%X \r\n\n", index);
    		Cy_SCB_I2C_MasterSendStop(i2c_HW, 100UL, &i2cContext);
    		Cy_SysLib_Delay(50);
    	}
    }
}
