#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

#define UART_INTR_NUM        ((IRQn_Type) scb_1_interrupt_IRQn)
#define UART_INTR_PRIORITY   (7U)

uint8_t rx_buf[32];
uint16_t dataRead;
uint32_t rx_len = 32;

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
		txcounter++;
	}

	if((event & CY_SCB_RX_INTR_NOT_EMPTY) == CY_SCB_RX_INTR_NOT_EMPTY)
	{
		rxcounter++;
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

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
 	sprintf(txBuffer, "\x1b[2J\x1b[;H");

 	Cy_SCB_UART_Transmit(UART_HW, txBuffer, 12,&UART_context);

 	while((Cy_SCB_UART_GetTransmitStatus(UART_HW,&UART_context) & CY_SCB_UART_TRANSMIT_ACTIVE) != 0){}

 	sprintf(txBuffer, "******************  FreeRTOS UART Example  ****************** \r\n");

 	Cy_SCB_UART_Transmit(UART_HW, txBuffer, strcspn(txBuffer,"\r\n")+2,&UART_context);
 	while((Cy_SCB_UART_GetTransmitStatus(UART_HW,&UART_context) & CY_SCB_UART_TRANSMIT_ACTIVE) != 0){}

    for (;;)
    {
        sprintf(txBuffer, "Tx = %d\tRx = %d\r\n", txcounter, rxcounter);

        Cy_SCB_UART_Transmit(UART_HW, &txcounter, strcspn(txBuffer,"\r\n")+2,&UART_context);
        printf("Data Tx = %s",txBuffer);
    	while((Cy_SCB_UART_GetTransmitStatus(UART_HW,&UART_context) & CY_SCB_UART_TRANSMIT_ACTIVE) != 0){}

        Cy_SCB_UART_Receive(UART_HW, &rxcounter, 1,&UART_context);
        printf("Data Rx = %s\r\n",rxBuffer);
    	while((Cy_SCB_UART_GetReceiveStatus	(UART_HW,&UART_context) & CY_SCB_UART_RECEIVE_ACTIVE) != 0){}
    }
}
