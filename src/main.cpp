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

/* 
FOR SPI
SDA/SDI/MOSI/TX
SDO/MISO/RX
SCL/CLK/SCLK
CS/CSn
    const uint spi0_sdo_rx = 16;  
    const uint spi0_cs_MS5611 = 17;
    const uint spi0_sclk = 18;
    const uint spi0_sdi_tx = 19;

FOR I2C
    const uint sda0_pin = 16;
    const uint scl0_pin = 17;
    const uint sda1_pin = 18;
    const uint scl1_pin = 19;
*/


int main() {

    // Pins
    const uint led_pin = 25;
    // const uint sda0_pin = 16;
    // const uint scl0_pin = 17;
    // const uint sda1_pin = 18;
    // const uint scl1_pin = 19;

    const uint spi0_sdo_rx = 16;  
    const uint spi0_sclk = 18;
    const uint spi0_sdi_tx = 19;
    const uint spi0_cs_MS5611 = 17;


    // Variables
    //i2c_inst_t *i2c0_ptr = i2c0;
    //i2c_inst_t *i2c1_ptr = i2c1;
    spi_inst_t *spi0_ptr = spi0;
    //spi_inst_t *spi1_ptr = spi1;
    
    // Initialize 
    stdio_init_all();
    // i2c_setup(i2c0_ptr, sda0_pin, scl0_pin);
    spi_setup(spi0_ptr, 
        spi0_cs_MS5611, 
        spi0_sclk, 
        spi0_sdi_tx, 
        spi0_sdo_rx, 
        SPI_FREQ_1MHZ);
    
    // Workaround: Perform throw-away read to make SCK idle high
    uint8_t data[1];
    spi_reg_read(spi0_ptr, spi0_cs_MS5611, REG_DEVID, data, 1);

    
	spi_reg_write(spi0_ptr, MS56xx_ADDR, MS56xx_D1_OSR_4096, 0x00);     // Start conversion of the pressure sensor

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