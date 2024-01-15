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
*/

#ifndef _bmp280_h_
#define _bmp280_h_

#include "cyhal.h"
#include "inttypes.h"
#include "stdio.h"
#include "stdarg.h"

#ifndef VA_MACRO
    #define NUM_ARGS_(_1, _2, _3, _4, _5, _6, _7, _8, TOTAL, ...) TOTAL
    #define NUM_ARGS(...) NUM_ARGS_(__VA_ARGS__, 6, 5, 4, 3, 2, 1, 0)
    #define CONCATE_(X, Y) X##Y
    #define CONCATE(MACRO, NUMBER)  CONCATE_(MACRO, NUMBER)
    #define VA_MACRO(MACRO, ...)    CONCATE(MACRO, NUM_ARGS(__VA_ARGS__))(__VA_ARGS__)
#endif

#define I2C_BUFFER_SIZE (10u)
#define BMP280_ADDR 0x76
#define BMP280_ID   0x58
#define BMP280_default_timeout (500u)

#define BMP280_SUCCESS          0ul
#define BMP280_ERROR            1ul
#define BMP280_ERROR_RW_Func    2ul
#define BMP280_ERROR_TIMEOUT    3ul

// sensor struct
typedef struct {
  uint16_t dig_T1; /**< dig_T1 cal register. */
  int16_t dig_T2;  /**<  dig_T2 cal register. */
  int16_t dig_T3;  /**< dig_T3 cal register. */

  uint16_t dig_P1; /**< dig_P1 cal register. */
  int16_t dig_P2;  /**< dig_P2 cal register. */
  int16_t dig_P3;  /**< dig_P3 cal register. */
  int16_t dig_P4;  /**< dig_P4 cal register. */
  int16_t dig_P5;  /**< dig_P5 cal register. */
  int16_t dig_P6;  /**< dig_P6 cal register. */
  int16_t dig_P7;  /**< dig_P7 cal register. */
  int16_t dig_P8;  /**< dig_P8 cal register. */
  int16_t dig_P9;  /**< dig_P9 cal register. */
} bmp280_calib_data;

typedef struct config_t_f {
    /** Inactive duration (standby time) in normal mode */
    unsigned int t_sb : 3;
    /** Filter settings */
    unsigned int filter : 3;
    /** Unused - don't set */
    unsigned int none : 1;
    /** Enables 3-wire SPI */
    unsigned int spi3w_en : 1;
    /** Used to retrieve the assembled config register's byte value. */
  }config_t;

typedef struct ctrl_meas {
    /** Temperature oversampling. */
    unsigned int osrs_t : 3;
    /** Pressure oversampling. */
    unsigned int osrs_p : 3;
    /** Device mode */
    unsigned int mode : 2;
    /** Used to retrieve the assembled ctrl_meas register's byte value. */
  }ctrl_meas_t;




uint32_t BMP280_readValue(float* temperature, float *pressure, uint32_t timeout);


typedef struct{
    bmp280_calib_data data_calibration;
    uint8_t addr;
    cyhal_i2c_t *obj;
//    uint32_t (*read_bytes)(cyhal_i2c_t *obj, uint16_t addr, uint16_t reg, uint16_t mem_reg_size, uint8_t *val, uint8_t dataLen, uint32_t timeout);
//    uint32_t (*write_bytes)(cyhal_i2c_t *obj, uint8_t addr,uint8_t *data, uint8_t dataLen, uint32_t timeout, bool send_stop);

//    uint32_t (*init)(cyhal_i2c_t *obj, uint16_t addr, uint32_t (*readFunction)(cyhal_i2c_t *objR, uint16_t,uint16_t, uint16_t, uint8_t*,uint16_t, uint32_t), uint32_t (*writeFunction)(cyhal_i2c_t *objW, uint16_t, uint8_t*, uint16_t, uint32_t, bool));
    uint32_t (*init)(cyhal_i2c_t *obj, uint16_t addr);
    uint32_t (*readValue)(float* temperature, float*pressure, uint32_t timeout);
    uint32_t (*temp_config) (void);
    uint32_t (*load_calibration)(void);
    uint8_t  (*read_DI) (void);

}bmp280_obj;

extern bmp280_obj bmp280;


#endif
/* [] END OF FILE */
