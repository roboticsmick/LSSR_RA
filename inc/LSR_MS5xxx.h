/*
 * MS5xx.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#ifndef _MS5xx_H
#define _MS5xx_H

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// MS5611 I2C device address
static const uint8_t MS5xx_ADDR = 0x77; //0b1110111

// MS5611 device commands
static const uint8_t MS5xx_RESET_COMMAND = 0x1E;
static const uint8_t MS5xx_START_PRESSURE_ADC_CONVERSION = 0x40;
static const uint8_t MS5xx_START_TEMPERATURE_ADC_CONVERSION = 0x50;
static const uint8_t MS5xx_READ_ADC = 0x00;
static const uint8_t MS5xx_CONVERSION_OSR_MASK = 0x0F;

// MS5611 commands
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_0 = 0xA0;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_1 = 0xA2;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_2 = 0xA4;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_3 = 0xA6;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_4 = 0xA8;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_5 = 0xAA;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_6 = 0xAC;
static const uint8_t MS5xx_PROM_ADDRESS_READ_ADDRESS_7 = 0xAE;

static const int16_t MS5xx_CONVERSION_TIME_OSR_256 = 1000;
static const int16_t MS5xx_CONVERSION_TIME_OSR_512 = 2000;
static const int16_t MS5xx_CONVERSION_TIME_OSR_1024 = 3000;
static const int16_t MS5xx_CONVERSION_TIME_OSR_2048 = 5000;
static const int16_t MS5xx_CONVERSION_TIME_OSR_4096 = 9000;

/* ************************************************ */
/* Structures */
/* ************************************************ */

/*! \brief recursive mutex instance
 * \ingroup MS56xx
 */
typedef struct MS56xx_data {
    float value;
    uint8_t count;        
    int16_t array[30];
} MS56xx_data_t;

/* ************************************************ */
/* Functions */
/* ************************************************ */

/*! \brief Brief explanation
 *  \ingroup group type
 *
 * Detailed explanation
 *
 * \param value1 value comment
 * \param value2 value comment
 * \return return comments
 */
extern void FuncName(MS56xx_data_t *sensordata, int int_val, float float_val);

/* ************************************************ */
/* Heading */
/* ************************************************ */

#ifdef __cplusplus
}
#endif
#endif