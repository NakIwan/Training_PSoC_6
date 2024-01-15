/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
 * andri setyabudi
 * andhie.13@gmail.com
 *
 * Edited by : Ridwan Bahari
 * E-mail    : ridwanbahari236@gmail.com
*/

#include "BMP280.h"


unsigned int config_get(config_t cfg){
    return (cfg.t_sb << 5) | (cfg.filter << 2) | cfg.spi3w_en;
}
unsigned int meas_get(ctrl_meas_t meas) {
    return (meas.osrs_t << 5) | (meas.osrs_p << 2) | meas.mode;
}

uint32_t  BMP280_temp_configure(void)
{
    uint32_t status;
    config_t t_cfg ={
            .t_sb   = 0x04,
            .filter = 0x04,
    };
    ctrl_meas_t t_meas = {
            .mode   = 0x03,
            .osrs_t = 0x02,
            .osrs_p = 0x05,
    };
    uint8_t buf[4];
    buf[0] = 0xF4;
    buf[1] = meas_get(t_meas);
    buf[2] = 0xF5;
    buf[3] = config_get(t_cfg);
    status = cyhal_i2c_master_write(bmp280.obj, bmp280.addr, buf, 4,1000,true);
    return status;
}

uint32_t BMP280_read_calibration(void)
{
    uint32_t status;
    uint8_t buffer_i2c[2];

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x88, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_T1 = (uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]);
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x8A, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_T2 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x8C, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_T3 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x8E, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P1 = (uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]);
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x90, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P2 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x92, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P3 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x94, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P4 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x96, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P5 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x98, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P6 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x9A, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P7 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x9C, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P8 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    else
        return status;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0x9E, 1, buffer_i2c, 2, 1000);
    if( status == BMP280_SUCCESS )
        bmp280.data_calibration.dig_P9 = (int16_t)((uint16_t)(buffer_i2c[1]) << 8 | (uint16_t)(buffer_i2c[0]));
    return status;
}

static uint8_t BMP280_read_ID(void)
{
    uint8_t ret = 0xff;

    if( cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0xD0, 1, &ret, 1, 1000) == BMP280_SUCCESS)
        return ret;
    else
        return 0xff;
}

uint32_t BMP280_readValue(float* temperature, float*pressure, uint32_t timeout)
{
    uint32_t status;
    uint8_t buffer_i2c[4];
    int32_t adc_T; int64_t var1, var2;

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0xFA, 1, buffer_i2c, 3, 1000);
    adc_T = (uint32_t)(buffer_i2c[0]) << 16 | (uint32_t)(buffer_i2c[1]) << 8 | (uint32_t)(buffer_i2c[2]);
    adc_T >>= 4;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280.data_calibration.dig_T1 << 1))) * ((int32_t)bmp280.data_calibration.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280.data_calibration.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280.data_calibration.dig_T1))) >> 12) * ((int32_t)bmp280.data_calibration.dig_T3)) >> 14;
    float T = ( ((var1 + var2) * 5 + 128) >> 8 ) ;
    *temperature =  (float)(T / 100);

    status = cyhal_i2c_master_mem_read(bmp280.obj, bmp280.addr, 0xF7, 1, buffer_i2c, 3, 1000);
    adc_T = (uint32_t)(buffer_i2c[0]) << 16 | (uint32_t)(buffer_i2c[1]) << 8 | (uint32_t)(buffer_i2c[2]);

    adc_T >>= 4;
    var1 = ((int64_t)(var1 + var2)) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280.data_calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280.data_calibration.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280.data_calibration.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280.data_calibration.dig_P3) >> 8) +
         ((var1 * (int64_t)bmp280.data_calibration.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280.data_calibration.dig_P1) >> 33;

    if (var1 == 0)
        *pressure = 0; // avoid exception caused by division by zero
    else
    {
        int64_t p = 1048576 - adc_T;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((int64_t)bmp280.data_calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((int64_t)bmp280.data_calibration.dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280.data_calibration.dig_P7) << 4);
        *pressure = (float)p / 256;
    }
    return status;
}

uint32_t BMP280_init(cyhal_i2c_t *obj, uint16_t addr)
{
    uint32_t result = 0ul;
    bmp280.addr = addr;
    bmp280.obj  = obj;

    if(bmp280.read_DI() != BMP280_ID )
        return BMP280_ERROR;
    result = bmp280.load_calibration();
    if( result!= BMP280_SUCCESS )
        return result;
    result = bmp280.temp_config();
    return result;
}


bmp280_obj bmp280 = {
    .data_calibration 	= {0,},
    .addr             	= BMP280_ADDR,
    .init               = BMP280_init,
    .readValue          = BMP280_readValue,
    .temp_config        = BMP280_temp_configure,
    .load_calibration   = BMP280_read_calibration,
    .read_DI            = BMP280_read_ID,
};
/* [] END OF FILE */
