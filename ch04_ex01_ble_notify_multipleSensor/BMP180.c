/*
 * BMP180.c
 *
 *  Created on: 12 Okt 2023
 *      Author: dwisetyabudi
 */

#include "BMP180.h"

static int32_t calculateB5(int32_t ut) {
  int32_t X1 =
      (ut - (int32_t)bmp180.data_calibration.ac6) * ((int32_t)bmp180.data_calibration.ac5) >> 15;
  int32_t X2 =
      ((int32_t)bmp180.data_calibration.mc << 11) / (X1 + (int32_t)bmp180.data_calibration.md);
  return X1 + X2;
}

static uint32_t BMP180_read_ID(void)
{
    uint32_t ret = 0xff;
    if( bmp180.read_bytes == NULL )
        return ret;
    if( BMP180_SUCCESS == bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CHIPID, &ret,1, BMP180_default_timeout) )
        return ret;
    else
        return 0xff;
}

static uint32_t BMP180_read_calibration(void)
{
    uint32_t status;
    uint8_t buffer_i2c[2];
    if( bmp180.read_bytes == NULL )
        return BMP180_ERROR_RW_Func;
    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_AC1, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.ac1 = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;
    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_AC2, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.ac2 = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_AC3, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.ac3 = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_AC4, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.ac4 = (uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]);
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_AC5, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.ac5 = ((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_AC6, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.ac6 = ((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_B1, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.b1 = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_B2, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.b2 = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_MB, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.mb = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_MC, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.mc = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    else
        return status;

    status = bmp180.read_bytes(bmp180.addr, (uint8_t)BMP180_REGISTER_CAL_MD, buffer_i2c, 2, BMP180_default_timeout);
    if( status == BMP180_SUCCESS )
        bmp180.data_calibration.md = (int16_t)((uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]));
    return status;
}

static uint32_t BMP180_init(bmp180_mode_t mode, uint8_t addr, uint32_t (*readFunction)(uint8_t,uint8_t, uint8_t*, uint8_t, uint32_t), uint32_t (*writeFunction)(uint8_t, uint8_t*, uint8_t, uint32_t))
{
    uint32_t result = 0ul;
    bmp180.addr = addr;
    bmp180.read_bytes   = readFunction;
    bmp180.write_bytes  = writeFunction;

    /* Mode boundary check */
	if ((mode > BMP180_MODE_ULTRAHIGHRES) || (mode < 0))
		bmp180.mode = BMP180_MODE_ULTRAHIGHRES;
	else
		bmp180.mode = mode;

    if(bmp180.read_ID() != BMP180_ID )
        return BMP180_ERROR;
    result = bmp180.load_calibration();
    return result;
}

uint32_t BMP180_readValue(float* temperature, float*pressure, uint32_t timeout)
{
    uint32_t status;
    uint8_t buffer_i2c[4];
    int16_t adc_T, B5;
    int32_t adc_P;
    if( bmp180.read_bytes == NULL )
        return BMP180_ERROR_RW_Func;
    // request temperature data
    buffer_i2c[0] = BMP180_REGISTER_CONTROL;
    buffer_i2c[1] = BMP180_REGISTER_READTEMPCMD;
    status = bmp180.write_bytes(bmp180.addr, buffer_i2c, 2, timeout);
    if( status == BMP180_SUCCESS )
    {
    	// waiting for status event
    	buffer_i2c[0]=0xff;
    	do{
    		status = bmp180.read_bytes(bmp180.addr, BMP180_REGISTER_CONTROL, buffer_i2c, 1, timeout);
    		buffer_i2c[0] &= 1<<5;
    	}while(status==BMP180_SUCCESS && buffer_i2c[0]);
    	status = bmp180.read_bytes(bmp180.addr, BMP180_REGISTER_TEMPDATA, buffer_i2c, 2, timeout);
    }else
    	return status;

    adc_T = (uint16_t)(buffer_i2c[0]) << 8 | (uint16_t)(buffer_i2c[1]) ;
    B5 = calculateB5(adc_T);
    *temperature = (B5 + 8) >> 4;
    *temperature /= 10;

    // request pressure data
    buffer_i2c[0] = BMP180_REGISTER_CONTROL;
	buffer_i2c[1] = BMP180_REGISTER_READPRESSURECMD + (bmp180.mode << 6);
	status = bmp180.write_bytes(bmp180.addr, buffer_i2c, 2, timeout);
	if( status == BMP180_SUCCESS )
	{
		// waiting for status event
		buffer_i2c[0]=0xff;
		do{
			status = bmp180.read_bytes(bmp180.addr, BMP180_REGISTER_CONTROL, buffer_i2c, 1, timeout);
			buffer_i2c[0] &= 1<<5;
		}while(status==BMP180_SUCCESS && buffer_i2c[0]);
		status = bmp180.read_bytes(bmp180.addr, BMP180_REGISTER_PRESSUREDATA, buffer_i2c, 3, timeout);
	}else
		return status;
	adc_P = (uint32_t)(buffer_i2c[0]) << 16 | (uint32_t)(buffer_i2c[1]) << 8 | (uint32_t)(buffer_i2c[2]);
	adc_P >>= (8 - bmp180.mode);

	/* Pressure compensation */
	int32_t  b6 = B5 - 4000;
	int32_t  x1 = (bmp180.data_calibration.b2 * ((b6 * b6) >> 12)) >> 11;
	int32_t  x2 = (bmp180.data_calibration.ac2 * b6) >> 11;
	int32_t  x3 = x1 + x2;
	int32_t  b3 = (((((int32_t)bmp180.data_calibration.ac1) * 4 + x3) << bmp180.mode) + 2) >> 2;
	  x1 = (bmp180.data_calibration.ac3 * b6) >> 13;
	  x2 = (bmp180.data_calibration.b1 * ((b6 * b6) >> 12)) >> 16;
	  x3 = ((x1 + x2) + 2) >> 2;
	uint32_t  b4 = (bmp180.data_calibration.ac4 * (uint32_t)(x3 + 32768)) >> 15;
	uint32_t  b7 = ((uint32_t)(adc_P - b3) * (50000 >> bmp180.mode));

	  if (b7 < 0x80000000) {
		  adc_P = (b7 << 1) / b4;
	  } else {
		  adc_P = (b7 / b4) << 1;
	  }

	  x1 = (adc_P >> 8) * (adc_P >> 8);
	  x1 = (x1 * 3038) >> 16;
	  x2 = (-7357 * adc_P) >> 16;
	  *pressure = adc_P + ((x1 + x2 + 3791) >> 4);
    return status;
}

bmp180_obj bmp180 = {
    .data_calibration = {0,},
    .addr             = BMP180_ADDR,
	.mode			  = BMP180_MODE_ULTRAHIGHRES,
    .read_bytes       = NULL,
    .write_bytes      = NULL,

    .init               = BMP180_init,
    .readValue          = BMP180_readValue,
    .load_calibration   = BMP180_read_calibration,
    .read_ID            = BMP180_read_ID,
};

