#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define UART_INTR_NUM        ((IRQn_Type) scb_1_interrupt_IRQn)
#define UART_INTR_PRIORITY   (7U)

uint8_t rx_buf[32];
uint16_t dataRead;
uint32_t rx_len = 32;
uint8_t flag = false;
char txBuffer[100];
char rxBuffer[5];
int txcounter = 0;
int rxcounter = 0;

cy_stc_scb_uart_context_t UART_context;

void UART_isr(void){
	Cy_SCB_UART_Interrupt(UART_HW, &UART_context);
}

void UART_Handler(uint32_t event){

	if((event & CY_SCB_TX_INTR_LEVEL ) == 	CY_SCB_TX_INTR_LEVEL )
	{
    	uint8_t cmd = 0X55;
        if(Cy_SCB_UART_Transmit(UART_HW, &cmd,2,&UART_context) == CY_RSLT_SUCCESS)Cy_SCB_UART_Receive(UART_HW, &rx_buf, 1,&UART_context);
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

    cy_stc_sysint_t uartIntrConfig =
    {
        .intrSrc      = UART_INTR_NUM,
        .intrPriority = UART_INTR_PRIORITY,
    };

    /* Enable global interrupts */
    __enable_irq();

    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if(result == CY_RSLT_SUCCESS)printf("\r\nHello US100 Ultrasonic\r\n");

    result = Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    if(result == CY_RSLT_SUCCESS)printf("Success initialization UART\r\n\n");
    Cy_SCB_UART_Enable(UART_HW);

    (void) Cy_SysInt_Init(&uartIntrConfig, &UART_isr);
    NVIC_EnableIRQ(UART_INTR_NUM);

   	Cy_SCB_UART_RegisterCallback(UART_HW, (cy_cb_scb_uart_handle_events_t)UART_Handler, &UART_context);
   	Cy_SCB_SetRxInterruptMask(UART_HW, CY_SCB_RX_INTR_NOT_EMPTY);

   	/* Enabling the SCB block for UART operation */
   	Cy_SCB_UART_Enable(UART_HW);

    for (;;)
    {
                uint16_t dataUs100 = (uint16_t)rx_buf[0] << 8 | rx_buf[1];
                printf("Data Rx = %d\r\n",dataUs100);
    }
}
