/*
 * MS56xxx.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#ifndef _MS56xx_H
#define _MS56xx_H

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// MS5611 I2C device address
static const uint8_t MS56xx_ADDR = 0x77;
static const uint8_t MS56xx_ADDR_HIGH = 0x76; 

// MS5611 device commands
static const uint8_t MS56xx_RESET_COMMAND = 0x1E;
static const uint8_t MS56xx_START_PRESSURE_ADC_CONVERSION = 0x40;
static const uint8_t MS56xx_START_TEMPERATURE_ADC_CONVERSION = 0x50;
static const uint8_t MS56xx_READ_ADC = 0x00;
static const uint8_t MS56xx_CONVERSION_OSR_MASK = 0x0F;

// MS5611 commands
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_0 = 0xA0;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_1 = 0xA2;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_2 = 0xA4;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_3 = 0xA6;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_4 = 0xA8;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_5 = 0xAA;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_6 = 0xAC;
static const uint8_t MS56xx_PROM_ADDRESS_READ_ADDRESS_7 = 0xAE;

static const uint8_t MS56xx_D1_OSR_256 = 0x40;
static const uint8_t MS56xx_D1_OSR_512 = 0x42;
static const uint8_t MS56xx_D1_OSR_1024 = 0x44;
static const uint8_t MS56xx_D1_OSR_2048 = 0x46;
static const uint8_t MS56xx_D1_OSR_4096 = 0x48;

static const uint8_t MS56xx_D2_OSR_256 = 0x50;
static const uint8_t MS56xx_D2_OSR_512 = 0x52;
static const uint8_t MS56xx_D2_OSR_1024 = 0x54;
static const uint8_t MS56xx_D2_OSR_2048 = 0x56;
static const uint8_t MS56xx_D2_OSR_4096 = 0x58;

static const int16_t MS56xx_CONVERSION_TIME_OSR_256 = 1000;
static const int16_t MS56xx_CONVERSION_TIME_OSR_512 = 2000;
static const int16_t MS56xx_CONVERSION_TIME_OSR_1024 = 3000;
static const int16_t MS56xx_CONVERSION_TIME_OSR_2048 = 5000;
static const int16_t MS56xx_CONVERSION_TIME_OSR_4096 = 9000;

/* ************************************************ */
/* Structures */
/* ************************************************ */

/*! \brief recursive mutex instance
 * \ingroup MS56xx
 */
typedef struct MS56xx_data {
    uint8_t devAddr; // I2C device adress
    uint16_t C1, C2, C3, C4, C5, C6; // Calibration data
    uint32_t D1, D2; // Raw measurement data
    float TEMP; // Calculated temperature
    float PRES; // Calculated pressure
} MS56xx_data_t;

/* ************************************************ */
/* Functions */
/* ************************************************ */

/*! \brief Breif explanation
 *  \ingroup MS56xx Sensor
 *
 * Detailed explanation
 *
 * \param value1 value comment
 * \param value2 value comment
 * \return return comments
 */
extern bool MS56xx_Init(i2c_inst_t *i2c);

/*! \brief Breif explanation
 *  \ingroup MS56xx Sensor
 *
 * Detailed explanation
 *
 * \param i2c value comment
 * \param MS56xx_data value comment
 * \return return comments
 */
extern bool MS56xx_Read(i2c_inst_t *i2c, MS56xx_data_t *MS56xx_data);

#ifdef __cplusplus
}
#endif
#endif