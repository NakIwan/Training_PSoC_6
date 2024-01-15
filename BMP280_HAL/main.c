#include "cyhal.h"
#include "cybsp.h"
#include "BMP280.h"
#include "cy_retarget_io.h"

cyhal_i2c_t i2c;

cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
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

    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Initialize i2c */
    result = cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cyhal_i2c_configure(&i2c, &i2c_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

	uint8_t hasil;
	for (uint8_t i = 1; i < 0x7f; i++) {
	    if (cyhal_i2c_master_write(&i2c, i, &hasil, 0,100,true) == 0)
	      printf(" found device on address: 0x%x \r\n", i);
	  }

    if (bmp280.init(&i2c, BMP280_ADDR) == BMP280_SUCCESS)
      printf("Device Found !\r\n");

    float temp,press;
    for (;;)
    {
    	BMP280_readValue(&temp, &press, 100);
    	printf("Temp : %0.2f\tPress : %0.2f\r\n",temp,press);


    }
}
