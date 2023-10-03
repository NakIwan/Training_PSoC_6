#include "cyhal.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"

    volatile uint32 SysTickCount;
    void SysTickCallback(void)
    {
    SysTickCount++;
    }

    uint32_t getTickCount(){return SysTickCount;}


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

    /* Initialize the User LED */
    result = cyhal_gpio_init(P6_3, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1U);
	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}
    result = cyhal_gpio_init(P7_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1U);
	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	/* Initialize the user button */
	result = cyhal_gpio_init(P0_4, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1U);
	/* GPIO init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);//Initialization UART on HAL
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU, (SystemCoreClock/1000)-1);
    Cy_SysTick_SetCallback(0,SysTickCallback);
    Cy_SysTick_Enable();

    /* Enable global interrupts */
    __enable_irq();

    for (;;)
    {

        static const uint32_t DOUBLE_GAP_MILLIS_MAX = 250 ;
        static const uint32_t LONG_MILLIS_MIN = 300 ;

        static uint32_t button_down_ts = 0 ;
        static uint32_t button_up_ts = 0 ;

        static bool double_pending = false ;
        static bool long_press_pending = false ;
        static bool button_down = false ;

        uint32_t DEBOUNCE_MILLIS = 20 ;
        bool buttonstate =! cyhal_gpio_read(P0_4);
        uint32_t buttonstate_ts = getTickCount() ;
        uint32_t now = getTickCount();

        if( now - buttonstate_ts > DEBOUNCE_MILLIS )
        {
            if( buttonstate != cyhal_gpio_read(P0_4))
            {
                buttonstate = !buttonstate ;
                buttonstate_ts = now ;
            }
        }

        // If state changed...
        if( button_down != buttonstate )
        {
            button_down = !button_down ;
            if( button_down )
            {
                // Timestamp button-down
                button_down_ts = now ;
            }
            else
            {
                // Timestamp button-up
                button_up_ts = now ;

                // If double decision pending...
                if( double_pending )
                {
                    //button_event = DOUBLE_PRESS ;
                	printf("Double click\r\n");
                	for(int i = 0; i < 3; i++){
                    	cyhal_gpio_write(P6_3, 0);
                    	cyhal_system_delay_ms(200);
                    	cyhal_gpio_write(P6_3, 1);
                    	cyhal_system_delay_ms(200);
                	}
                    double_pending = false ;
                }
                else
                {
                    double_pending = true ;
                }

                // Cancel any long press pending
                long_press_pending = false ;
            }
        }
        // If button-up and double-press gap time expired, it was a single press
        if( !button_down && double_pending && now - button_up_ts > DOUBLE_GAP_MILLIS_MAX )
        {
            double_pending = false ;
            printf("Single click\r\n");
            for(int i = 0;i <= 2; i++ ){
            	cyhal_gpio_write(P7_1, 0);
            	cyhal_system_delay_ms(200);
            	cyhal_gpio_write(P7_1, 1);
            	cyhal_system_delay_ms(200);
            }
            //button_event = SINGLE_PRESS ;
        }
        // else if button-down for long-press...
        else if( !long_press_pending && button_down && now - button_down_ts == LONG_MILLIS_MIN )
        {
            //button_event = LONG_PRESS ;
            long_press_pending = false ;
            double_pending = false ;
            printf("Long press\r\n");
            for (int i = 0; i < 4; i++){
            	cyhal_gpio_write(P7_1, 0);
            }

        }

    }
}
