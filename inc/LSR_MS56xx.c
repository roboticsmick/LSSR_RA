/*
 * LSR_MS56xx.c
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "LSR_MS56xx.h"
#include "LSR_serial.h"

bool MS56xx_Init(i2c_inst_t *i2c) {

    uint8_t data[1];

    //  MS5611 to reset  taking measurements by setting Measure bit to high
    data[0] = 0x00;
    i2c_reg_write(i2c, MS56xx_ADDR, MS56xx_RESET_COMMAND, &data[0], 1);

    return true;
}

