#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "xensiv_dps3xx_mtb.h"

xensiv_dps3xx_t pressure_sensor;

cyhal_i2c_t i2c;
cyhal_i2c_cfg_t i2c_cfg = {
    .is_slave = false,
    .address = 0,
    .frequencyhal_hz = 400000
};

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);
    CY_ASSERT(result == CY_RSLT_SUCCESS);

    printf("DPS310 Pressure Sensor\r\n");

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
    //result = cyhal_i2c_master_read(&i2c, XENSIV_DPS3XX_I2C_ADDR_DEFAULT, NULL, 0, 150, true);
    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c, 0x77);
    if (result != CY_RSLT_SUCCESS)
    {
    	printf("Error initialization DPS310 Sensor \r\n");
    	CY_ASSERT(0);
    }

    for (;;)
    {
        /* Get the pressure and temperature data and print the results to the UART */
        float pressure, temperature;

        result = xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);
        if (result != CY_RSLT_SUCCESS)printf("Success read\r\n\n");
        printf("Pressure   : %f\r\n", pressure);
        printf("Temperature: %f\r\n\r\n", temperature);

        cyhal_system_delay_ms(100);
    }
}
