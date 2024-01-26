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
#include <stdio.h>

#include "cyhal.h"
#include "cybsp.h"

#include "mtb_ssd1306.h"

#include "u8g2/u8g2.h"
#include "u8g2/u8g2_support.h"

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
int main(void)
{
    cy_rslt_t result;
    // NanDbg-ADD
    cyhal_i2c_t i2c_obj;
    // NanDbg-Add
    u8g2_t u8g2_obj;

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

    /* Initialize the HAL I2C */
    result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL); 
    if (result != CY_RSLT_SUCCESS)
    {
        perror("Failed init I2C!!!\r\n");
        CY_ASSERT(0);
    }

    /* Initialize the OLED display */
    result = mtb_ssd1306_init_i2c(&i2c_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        perror("Failed init OLED!!!\r\n");
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* NanDbg-Add: initialize for U8 on display follow example on 
                   display-oled-ssd1306 API doc */
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2_obj, U8G2_R0, u8x8_byte_hw_i2c, u8x8_gpio_and_delay_cb);
    u8g2_InitDisplay(&u8g2_obj);
    /* Disable power save / wake up display */
    u8g2_SetPowerSave(&u8g2_obj, 0);
    /* Prepare display for printing */
    u8g2_SetFontPosCenter(&u8g2_obj);
    //NanDbg: TryFont: u8g2_SetFont(&u8g2_obj, u8g2_font_crox3h_tf);
    //NanDbg: TryFont: u8g2_SetFont(&u8g2_obj, u8g2_font_4x6_tf);
    u8g2_SetFont(&u8g2_obj, u8g2_font_lucasfont_alternate_tf);

    u8g2_ClearDisplay(&u8g2_obj);
    u8g2_ClearBuffer(&u8g2_obj);

    /* Try print a message */
    const char str_try[] = "NanDbg: Try disp U8g2";
    //const char str_try[] = "NanDbg: \r\nTry disp U8g2 with longer string";
    //const char str_try[] = "NanDbg: Try disp U8g2 with longer string";
    uint8_t width_str = u8g2_GetUTF8Width(&u8g2_obj, str_try);

    uint8_t width_disp = u8g2_GetDisplayWidth(&u8g2_obj);
    uint8_t height_disp = u8g2_GetDisplayHeight(&u8g2_obj);

    u8g2_DrawFrame(&u8g2_obj, 0, 0, width_disp, height_disp);

    u8g2_DrawStr(&u8g2_obj, (width_disp-width_str)/2, height_disp/2, str_try);
    u8g2_SendBuffer(&u8g2_obj);

    for (;;)
    {
    }
}

/* [] END OF FILE */
