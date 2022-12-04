
/*
 * ADXL3xxx.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL343.pdf
 *      
 *      https://www.digikey.com/en/maker/projects/raspberry-pi-pico-rp2040-i2c-example-with-micropython-and-cc/47d0c922b79342779cdbd4b37b7eb7e2
 *      https://www.digikey.be/en/maker/projects/raspberry-pi-pico-rp2040-spi-example-with-micropython-and-cc/9706ea0cf3784ee98e35ff49188ee045
 *  
 */


#ifndef SERIAL_COMMS_H
#define SERIAL_COMMS_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C address
static const uint8_t ADXL343_ADDR = 0x53;

// Registers
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;

// Other constants
static const uint8_t DEVID = 0xE5;
static const float SENSITIVITY_2G = 1.0 / 256;  // (g/LSB)
static const float EARTH_GRAVITY = 9.80665;     // Earth's gravity in [m/s^2]

/*! \brief recursive mutex instance
 * \ingroup mutex
 */
typedef struct ADXL3xx_data {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
} ADXL3xx_data_t;



/*! \brief Breif explanation
 *  \ingroup ADXL3xx Sensor
 *
 * Detailed explanation
 *
 * \param value1 value comment
 * \param value2 value comment
 * \return return comments
 */
extern bool ADXL3xxx_Init(i2c_inst_t *i2c);

/*! \brief Breif explanation
 *  \ingroup ADXL3xx Sensor
 *
 * Detailed explanation
 *
 * \param i2c value comment
 * \param ADXL3xx_data value comment
 * \return return comments
 */
extern bool ADXL3xxx_Read(i2c_inst_t *i2c, ADXL3xx_data_t *ADXL3xx_data);

#ifdef __cplusplus
}
#endif
#endif