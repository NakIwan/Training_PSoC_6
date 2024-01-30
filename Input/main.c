#include "cyhal.h"
#include "cybsp.h"
#include "cy_pdl.h"
#include "cy_retarget_io.h"

#define GPIO_INTERRUPT_PRIORITY (7u)

#define BTN_ENTER 	(P0_2)
#define BTN_BACK	(P0_3)

#define MAIN_SCREEN 		1
#define SET_CLOCK_SCREEN 	2
#define BMP280_SCREEN 		3
#define PASCO2_SCREEN 		4

#define SET_CLOCK	1
#define BMP280		2
#define PASCO2		3

uint8_t State, lastState;
uint8_t lastButtonState;
uint8_t	output;

uint8_t current_screen = 1;
uint8_t menu_item = 1;

bool menuCmd = false;
bool upCmd = false;
bool downCmd = false;
bool backCmd = false;
bool enterCmd = false;

unsigned long holdTime = 500, DBInterval = 50, RO_Time;
unsigned long timer1, duration = 1000, HeldTime;
unsigned long lastDebounceTime;

volatile uint32 SysTickCount;
void SysTickCallback(void)
{
	SysTickCount++;
}

uint32_t getTickCount(){return SysTickCount;}

static void button_isr(void *handler_arg, cyhal_gpio_event_t event)
{
	menuCmd = true;
}

cyhal_gpio_callback_data_t cb_data =
{
		.callback     = button_isr,
		.callback_arg = NULL
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

    		/* Initialize the User LED */
//    		result = cyhal_gpio_init(CYBSP_, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1U);
//    		/* GPIO init failed. Stop program execution */
//    		if (result != CY_RSLT_SUCCESS)
//    		{
//    			CY_ASSERT(0);
//    		}
//    		result = cyhal_gpio_init(P7_1, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1U);
//    		/* GPIO init failed. Stop program execution */
//    		if (result != CY_RSLT_SUCCESS)
//    		{
//    			CY_ASSERT(0);
//    		}

    		/* Initialize the user button */
    		result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1U);
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

    		cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
//    		cyhal_gpio_init(BTN_BACK, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
//    		cyhal_gpio_init(BTN_ENTER, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
//
//    		cyhal_gpio_register_callback(CYBSP_USER_BTN2, &cb_data);
//    		cyhal_gpio_enable_event(CYBSP_USER_BTN2, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);

    		for(;;){

    			output = 0;
    			uint8_t btnApp = cyhal_gpio_read(CYBSP_USER_BTN);
    			if ( btnApp != lastButtonState){
    				lastDebounceTime = getTickCount();
    				lastButtonState = btnApp;
    				timer1 = getTickCount();
    			}

    			RO_Time = getTickCount();
    			if (RO_Time > getTickCount())
    				timer1 = RO_Time;

    			while ((getTickCount() - timer1) <= duration){
    				if (btnApp != lastState){
    					if (btnApp == !State){
    						if ((getTickCount() - lastDebounceTime) >= DBInterval){
    							output++;
    							lastDebounceTime = getTickCount();
    						}
    					}
    					lastState = cyhal_gpio_read(CYBSP_USER_BTN);
    				}
    				btnApp = cyhal_gpio_read(CYBSP_USER_BTN);
    			}

    			if (btnApp == State && btnApp == lastButtonState)
    				if ((HeldTime = (getTickCount() - timer1)) > holdTime)
    					output = 100;

    			/* Up Command*/
    			if (output == 1 && upCmd == false){
    				upCmd = true;
    				if (upCmd && output == 1)
    				{
    					upCmd = false;
    					menu_item --;
    					if (menu_item < SET_CLOCK)
    						menu_item = PASCO2;
    				}
    			}

    			/* Down command*/
    			if (output == 2 && downCmd == false){
    				downCmd = true;
    				if (downCmd == true && output == 2)
    				{
    					downCmd = false;
    					menu_item ++;
    					if (menu_item > PASCO2)
    						menu_item = SET_CLOCK;
    				}
    			}

    			/* Back Command */
    			if (output == 3 && backCmd == false)
    			{
    				backCmd = true;
    				if (backCmd == true && output == 3){
    					backCmd = false;
    					if (current_screen == SET_CLOCK_SCREEN || current_screen == BMP280_SCREEN || current_screen == PASCO2_SCREEN)
    					{
    						current_screen = MAIN_SCREEN;
    					}
    				}
    			}

    			/* Enter Command*/
    			if (output == 100 && enterCmd == false){
    				enterCmd = true;
    				if (enterCmd == true && output == 100)
    				{
    					enterCmd = false;
    					if (current_screen == MAIN_SCREEN && menu_item == SET_CLOCK)
    						current_screen = SET_CLOCK_SCREEN;

    					else if (current_screen == MAIN_SCREEN && menu_item == BMP280)
    						current_screen = BMP280_SCREEN;

    					else if (current_screen == MAIN_SCREEN && menu_item == PASCO2)
    						current_screen = PASCO2_SCREEN;
    				}
    			}
    			printf("Output : %d\r\n",output);
    		}

}
