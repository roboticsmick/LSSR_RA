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
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "../inc/LSR_serial.h"
#include "../inc/LSR_ADXL3xx.h"
#include "../inc/LSR_MS56xx.h"

/* 
Upload from terminal instructions:
cp -a ./build/LSR_RA.uf2 /media/logic/RPI-RP2/ 
*/


int main() {

    // Pins
    const uint led_pin = 25;
    const uint sda0_pin = 16;
    const uint scl0_pin = 17;
    const uint sda1_pin = 18;
    const uint scl1_pin = 19;

    // Variables
    i2c_inst_t *i2c0_ptr = i2c0;
    //i2c_inst_t *i2c1_ptr = i2c1;
    
    //ADXL3xx_data_t ADXL3xx_data = {0,0,0,0,0,0};

    // Initialize 
    stdio_init_all();
    i2c_setup(i2c0_ptr, sda0_pin, scl0_pin);
    
    //ADXL3xxx_Init(i2c0_ptr);

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Loop forever
    while (true) {
        //ADXL3xxx_Read(i2c0_ptr, &ADXL3xx_data);
        // Blink LED
        printf("Blinking!\r\n");
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        sleep_ms(1000);
    }
}