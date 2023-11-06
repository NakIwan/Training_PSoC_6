/*
 * BMP180.h
 *
 *  Created on: 12 Okt 2023
 *      Author: dwisetyabudi
 */

#ifndef BMP180_H_
#define BMP180_H_

#include <limits.h>
#include <math.h>
#include "inttypes.h"
#include "stdio.h"

#define BMP180_ADDR (0x77)
#define BMP180_ID   (0x55)
#define BMP180_default_timeout (1000u)

#define BMP180_SUCCESS          0ul
#define BMP180_ERROR            1ul
#define BMP180_ERROR_RW_Func    2ul
#define BMP180_ERROR_TIMEOUT    3ul


typedef enum {
	BMP180_REGISTER_CAL_AC1 		= 0xAA,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC2 		= 0xAC,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC3 		= 0xAE,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC4 		= 0xB0,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC5 		= 0xB2,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_AC6 		= 0xB4,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_B1 			= 0xB6,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_B2 			= 0xB8,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MB 			= 0xBA,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MC 			= 0xBC,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CAL_MD 			= 0xBE,   //!< R   Calibration data (16 bits)
	BMP180_REGISTER_CHIPID 			= 0xD0,   //!< Register that contains the chip ID
	BMP180_REGISTER_VERSION 		= 0xD1,   //!< Register that contains the chip version
	BMP180_REGISTER_SOFTRESET 		= 0xE0,   //!< Register for doing a soft reset
	BMP180_REGISTER_CONTROL 		= 0xF4,   //!< Control register
	BMP180_REGISTER_TEMPDATA 		= 0xF6,   //!< Temperature data register
	BMP180_REGISTER_PRESSUREDATA 	= 0xF6,   //!< Pressure data register
	BMP180_REGISTER_READTEMPCMD 	= 0x2E,   //!< Read temperature control register value
	BMP180_REGISTER_READPRESSURECMD = 0x34    //!< Read pressure control register value
}bmp180_register_t;

typedef enum {
	BMP180_MODE_ULTRALOWPOWER 	= 0,
	BMP180_MODE_STANDARD  		= 1,
	BMP180_MODE_HIGHRES 		= 2,
	BMP180_MODE_ULTRAHIGHRES 	= 3
} bmp180_mode_t;

typedef struct {
  int16_t ac1;  //!< R calibration coefficient (16-bits)
  int16_t ac2;  //!< R calibration coefficient (16-bits)
  int16_t ac3;  //!< R calibration coefficient (16-bits)
  uint16_t ac4; //!< R calibration coefficient (16-bits)
  uint16_t ac5; //!< R calibration coefficient (16-bits)
  uint16_t ac6; //!< R calibration coefficient (16-bits)
  int16_t b1;   //!< R calibration coefficient (16-bits)
  int16_t b2;   //!< R calibration coefficient (16-bits)
  int16_t mb;   //!< R calibration coefficient (16-bits)
  int16_t mc;   //!< R calibration coefficient (16-bits)
  int16_t md;   //!< R calibration coefficient (16-bits)
} bmp180_calib_data;

typedef struct{
	bmp180_calib_data data_calibration;
    uint8_t addr;
    uint8_t mode;
    uint32_t (*read_bytes)(uint8_t addr, uint8_t reg, uint8_t *val, uint8_t dataLen, uint32_t timeout);
    uint32_t (*write_bytes)(uint8_t addr,uint8_t *data, uint8_t dataLen, uint32_t timeout);

    uint32_t (*init)(bmp180_mode_t mode, uint8_t addr, uint32_t (*readFunction)(uint8_t,uint8_t, uint8_t*, uint8_t, uint32_t), uint32_t (*writeFunction)(uint8_t, uint8_t*, uint8_t, uint32_t));
    uint32_t (*readValue)(float* temperature, float*pressure, uint32_t timeout);
    uint32_t (*load_calibration)(void);
    uint8_t  (*read_ID) (void);

}bmp180_obj;

extern bmp180_obj bmp180;
#endif /* BMP180_H_ */
