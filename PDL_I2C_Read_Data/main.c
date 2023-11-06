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
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define BMPAddr 0X76

cy_stc_scb_i2c_context_t i2cContext;
//cy_en_scb_i2c_status_t status;
uint8_t buffer;
uint32_t timeoutMs = 100UL;

uint32_t i2c_send_data(uint8_t addr, uint8_t*data, uint16_t len)
{
    if ( len == 0 )
        return 1;
    uint32_t result = 1;
    result = Cy_SCB_I2C_MasterSendStart(i2c_HW, addr, CY_SCB_I2C_WRITE_XFER, 50UL, &i2cContext);
    if(result == 0)
    {
        for(uint16_t i = 0; i<len; i++)
        {
            result = Cy_SCB_I2C_MasterWriteByte(i2c_HW,data[i], 5UL,&i2cContext);
            if(result!=0)
            break;
        }
    }
    Cy_SCB_I2C_MasterSendStop(i2c_HW,50UL, &i2cContext);
    Cy_SysLib_Delay(10);
    return result;
}

uint8_t i2c_read_data(uint8_t addr, uint8_t reg, uint8_t*pData, uint16_t len)
{
    if ( len == 0 )
        return 1;
    uint32_t result = 1;
    result = Cy_SCB_I2C_MasterSendStart(i2c_HW, addr, CY_SCB_I2C_WRITE_XFER, 50UL, &i2cContext);
    if(result == 0)
    {
        result = Cy_SCB_I2C_MasterWriteByte(i2c_HW,reg, 5UL,&i2cContext);
        if(result==0)
        {
            if( len>1 )
            {
                result = Cy_SCB_I2C_MasterSendReStart(i2c_HW, addr, CY_SCB_I2C_READ_XFER, 50UL, &i2cContext);
                if( result == 0)
                    result = Cy_SCB_I2C_MasterReadByte(i2c_HW,CY_SCB_I2C_NAK,pData, 5UL,&i2cContext);
            }else
            {
                for(uint16_t i = 0; i<len-1; i++)
                {
                    result = Cy_SCB_I2C_MasterReadByte(i2c_HW,CY_SCB_I2C_NAK,&pData[i], 5UL,&i2cContext);
                    if(result!=0)
                        break;
                }
                result = Cy_SCB_I2C_MasterReadByte(i2c_HW,CY_SCB_I2C_NAK,&pData[len-1], 5UL,&i2cContext);
            }

        }
    }
    Cy_SCB_I2C_MasterSendStop(i2c_HW,50UL, &i2cContext);
    return result;
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
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    result = Cy_SCB_I2C_Init(i2c_HW, &i2c_config, &i2cContext);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    result = Cy_SCB_I2C_Init(i2c_HW, &i2c_config, &i2cContext);
    if (result != CY_RSLT_SUCCESS)
      {
          CY_ASSERT(0);
      }

//    result = BMP280_init(BMPAddr, i2c_read_data, i2c_send_data);
//    if (result != CY_RSLT_SUCCESS)
//      {
//          CY_ASSERT(0);
//      }

    Cy_SCB_I2C_Enable(i2c_HW);

    for (;;)
    {

    uint8_t msb, lsb;// xlsb;
    //uint16_t UT, UP, d;
    uint8_t reg_dataTem[2] = {0XF4,0X2E};
// 	uint16_t reg_dataPress[2] = {0XF4, 0X34};
//    BMP280_temp_configure();
// 	if (i2c_read_data(BMPAddr, 0XF4, &d, 1)== CY_SCB_I2C_SUCCESS)
// 	     printf("datareceive = 0x%d\n\r", d);

 	//            uint8_t msb, lsb, xlsb;
 	//            uint8_t reg_data[2] = {0XF4,0X23};
 	//            if(i2c_send_data(0X76, reg_data, 2)==CY_SCB_I2C_SUCCESS)
 	//            {
 	//                printf("datasend\n\r");
 	//                uint8_t d = 0;
 	//                if (i2c_read_data(0X76, 0XF3, &d, 1)== CY_SCB_I2C_SUCCESS)
 	//                    printf("datareceive = 0x%d\n\r", d);
 	//            }

//Read uncompensated temperture
        if(i2c_send_data(BMPAddr, reg_dataTem, 2)==CY_SCB_I2C_SUCCESS){

        	printf("send 0X2E into 0XF4\n\r");
     		Cy_SCB_I2C_MasterSendStart(i2c_HW, BMPAddr, CY_SCB_I2C_WRITE_XFER, timeoutMs, &i2cContext);
    		Cy_SCB_I2C_MasterWriteByte (i2c_HW, 0X74, timeoutMs, &i2cContext);
        	Cy_SCB_I2C_MasterSendReStart(i2c_HW, BMPAddr, CY_SCB_I2C_READ_XFER, timeoutMs, &i2cContext);
        	result = Cy_SCB_I2C_MasterReadByte(i2c_HW, CY_SCB_I2C_ACK, &msb, timeoutMs, &i2cContext);
        	result = Cy_SCB_I2C_MasterReadByte(i2c_HW, CY_SCB_I2C_NAK, &lsb, timeoutMs, &i2cContext);
//        	UT = (msb << 8)|lsb;

        	printf("	MSB 0x%X \r\n", msb);
        	printf("	LSB 0x%X \r\n", lsb);
//        	printf("UT = %d \r\n\n", UT);

    		Cy_SCB_I2C_MasterSendStop(i2c_HW, timeoutMs, &i2cContext);
    		Cy_SysLib_Delay(1000UL);
        }

//Read uncompensated pressure
     	/*if(i2c_send_data(BMPAddr, reg_dataPress, 2)==CY_SCB_I2C_SUCCESS){

     		printf("send 0X34+(oss << 6) into 0XF4\n\r");
     		Cy_SCB_I2C_MasterSendStart(i2c_HW, BMPAddr, CY_SCB_I2C_WRITE_XFER, timeoutMs, &i2cContext);
    		Cy_SCB_I2C_MasterWriteByte (i2c_HW, 0XF6, timeoutMs, &i2cContext);
        	Cy_SCB_I2C_MasterSendReStart(i2c_HW, BMPAddr, CY_SCB_I2C_READ_XFER, timeoutMs, &i2cContext);
        	result = Cy_SCB_I2C_MasterReadByte(i2c_HW, CY_SCB_I2C_ACK, &msb, timeoutMs, &i2cContext);
        	result = Cy_SCB_I2C_MasterReadByte(i2c_HW, CY_SCB_I2C_ACK, &lsb, timeoutMs, &i2cContext);
        	result = Cy_SCB_I2C_MasterReadByte(i2c_HW, CY_SCB_I2C_NAK, &xlsb, timeoutMs, &i2cContext);
        	UP = (msb << 16 | lsb << 8 | xlsb)>>(8-oss);

        	printf("	MSB 0x%X \r\n", msb);
        	printf("	LSB 0x%X \r\n", lsb);
        	printf("	XLSB 0x%X \r\n", xlsb);
        	printf("UP = 0x%X \r\n\n", UP);

    		Cy_SCB_I2C_MasterSendStop(i2c_HW, timeoutMs, &i2cContext);
    		Cy_SysLib_Delay(1000UL);
     	}*/
    }
}
