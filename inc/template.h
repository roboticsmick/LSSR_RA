/*
 * SENSOR_NAME.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#ifndef _SENSOR_NAME_H
#define _SENSOR_NAME_H

#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/*  I2C address */
static const uint8_t SENSOR_NAME_ADDR = 0x01;

/*  Registers */
static const uint8_t REG_DEVID = 0x00;

/*  Other constants */
static const float SENSOR_NAME_VARIABLE = 1.0 / 256;  // (scale)

/*  Other functions */
#define BITSET(byte,bitnumber)   ((byte) |=  (1<<(bitnumber)))
#define BITCLEAR(byte,bitnumber) ((byte) &= ~(1<<(bitnumber)))
#define BITFLIP(byte,bitnumber)  ((byte) ^=  (1<<(bitnumber)))
#define BITCHECK(byte,bitnumber) ((byte) &   (1<<(bitnumber)))

/* ************************************************ */
/* Structures */
/* ************************************************ */

/*! \brief recursive mutex instance
 * \ingroup mutex
 */
typedef struct data  {
    float value;
    uint8_t count;        
    int16_t array[30];
} data_t;

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
extern void FuncName(data_t *sensordata, int int_val, float float_val);

/* ************************************************ */
/* Heading */
/* ************************************************ */



#ifdef __cplusplus
}
#endif
#endif