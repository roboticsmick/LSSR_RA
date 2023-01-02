/*
 * LSR_H3LIS200DLTR.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      https://www.st.com/content/ccc/resource/technical/document/datasheet/6c/e1/2a/9d/29/58/4e/25/DM00162194.pdf/files/DM00162194.pdf/jcr:content/translations/en.DM00162194.pdf
 * 
 */

#ifndef _H3LIS_H
#define _H3LIS_H

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// LSR_H3LIS200DLTR I2C device address
static const uint8_t H3LIS_ADDR = 0x00; 


#ifdef __cplusplus
}
#endif
#endif