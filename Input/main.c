#include "cyhal.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"

    uint32_t currentLED;
    uint32_t longPressTime = 1000, shortPressTime = 500;
    uint32_t pressTime, releaseTime;//, timePressLimit;
    int lastState = 0, currentState = 0, count = 0;//, buttonState = 0;
    bool isPressing = false, longDetected = false, mode = false;
    uint32_t ledOn = 1, ledOff = 2, backMode = 3;
    int lastSteadyState = 0;       // the previous steady state from the input pin
    int lastFlickerableState = 0;  // the previous flickerable state from the input pin
    uint32_t lastDebounceTime = 0;

    volatile uint32 SysTickCount;
    void SysTickCallback(void)
    {
    SysTickCount++;
    }

    uint32_t getTickCount(){return SysTickCount;}

    void switchLED() {
      currentLED = (currentLED + 1) % 4; // Pindah ke LED berikutnya
      cyhal_gpio_write(P6_3, 1);
      switch (currentLED) {
        case 0:
        	cyhal_gpio_write(P7_1, 0);
        	cyhal_system_delay_ms(2000);
        	cyhal_gpio_write(P7_1, 1);
        	cyhal_system_delay_ms(2000);
        	isPressing = false;
        	break;
        case 1:
        	cyhal_gpio_write(P7_1, 0);
        	cyhal_system_delay_ms(1000);
        	cyhal_gpio_write(P7_1, 1);
        	cyhal_system_delay_ms(1000);
        	isPressing = false;
        	break;
        case 2:
        	cyhal_gpio_write(P7_1, 0);
        	cyhal_system_delay_ms(500);
        	cyhal_gpio_write(P7_1, 1);
        	cyhal_system_delay_ms(500);
        	isPressing = false;
        	break;
        case 3:
        	cyhal_gpio_write(P7_1, 0);
        	cyhal_system_delay_ms(100);
        	cyhal_gpio_write(P7_1, 1);
        	cyhal_system_delay_ms(100);
        	isPressing = false;
        	break;
      }
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
	result = cyhal_gpio_init(CYBSP_USER_BTN1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1U);
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
    	currentState =! cyhal_gpio_read(P0_4);

    	if(lastState == 0 && currentState == 1){
    		pressTime = getTickCount();
    		isPressing = true;
    		longDetected = true;
    	}

    	else if (lastState == 1 && currentState == 0){
    		isPressing = false;
    		releaseTime = getTickCount();

        	uint32_t duration1 = releaseTime - pressTime;

        	if(duration1 < shortPressTime) switchLED();//printf ("Short Press is Detected\r\n");

    	}

    	if (isPressing == true && longDetected == true){
    		uint32_t duration = getTickCount() - pressTime;
    		if (duration > longPressTime){
    			longDetected = false;
    			printf("Long Press is Detected\r\n");
    			cyhal_gpio_write(P6_3, 0);
    		}
    	}

    	lastState = currentState;
    	/*if( getTickCount() - timePrev >= 1000)
    	    {
    	        uint32_t timeNow = getTickCount();
    	        printf("Duration : %d\r\n", (timeNow - timePrev));
    	        printf("Value : %u\r\n",getTickCount());
    	        timePrev = timeNow;
    	    }

    	//printf("Value : %u\r\n",getTickCount());*/
    }

}
