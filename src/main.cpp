/*
 * LSR_RA.c
 *  
 * Rocket Avionics for RP2040
 * 
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "../inc/LSR_serial.h"
#include "../inc/LSR_ADXL3xx.h"
#include "../inc/LSR_MS5xxx.h"

/* 
Upload from terminal instructions:
cp -a ./build/LSR_RA.uf2 /media/logic/RPI-RP2/ 
*/

/*******************************************************************************
 * Main
 */
int main() {

    // Pins
    const uint sda0_pin = 16;
    const uint scl0_pin = 17;

    // Variables
    i2c_inst_t *i2c0_ptr = i2c0;
    ADXL3xx_data_t ADXL3xx_data = {0,0,0,0,0,0};

    // Initialize 
    stdio_init_all();
    i2c_setup(i2c0_ptr, sda0_pin, scl0_pin);
    ADXL3xxx_Init(i2c0_ptr);

    // Loop forever
    while (true) {
        ADXL3xxx_Read(i2c0_ptr, &ADXL3xx_data);
        sleep_ms(100);
    }
}