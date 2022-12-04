/*
 * LSR_serial.c
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "LSR_serial.h"

// Write 1 byte to the specified register
bool i2c_setup(i2c_inst_t *i2c, const uint sda_pin, const uint scl_pin) {

    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    
    return true;
}

// Write 1 byte to the specified register
int i2c_reg_write(  i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t msg[nbytes + 1];

    // Check to make sure caller is sending 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Append register address to front of data packet
    msg[0] = reg;
    for (int i = 0; i < nbytes; i++) {
        msg[i + 1] = buf[i];
    }

    // Write data to register(s) over I2C
    i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

    return num_bytes_read;
}


// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int i2c_reg_read(  i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes) {

    int num_bytes_read = 0;

    // Check to make sure caller is asking for 1 or more bytes
    if (nbytes < 1) {
        return 0;
    }

    // Read data from register(s) over I2C
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

    return num_bytes_read;
}
bool spi_setup(spi_inst_t *spi,
                const uint cs_pin,
                const uint sck_pin,
                const uint sdi_pin, 
                const uint sdo_pin) {

    // Initialize CS pin high
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);

    // Initialize SPI port at 1 MHz
    spi_init(spi, 1000 * 1000);
    
    // Set SPI format
    spi_set_format( spi,   // SPI instance
                    8,      // Number of bits per transfer
                    1,      // Polarity (CPOL)
                    1,      // Phase (CPHA)
                    SPI_MSB_FIRST);

    // Initialize SPI pins
    gpio_set_function(sck_pin, GPIO_FUNC_SPI);
    gpio_set_function(sdi_pin, GPIO_FUNC_SPI);
    gpio_set_function(sdo_pin, GPIO_FUNC_SPI);

}

bool spi_cs_setup(const uint cs_pin) {
    // Initialize CS pin high
    gpio_init(cs_pin);
    gpio_set_dir(cs_pin, GPIO_OUT);
    gpio_put(cs_pin, 1);
}

void spi_reg_write(spi_inst_t *spi, 
                const uint cs, 
                const uint8_t reg, 
                const uint8_t data) {
    uint8_t msg[2];
                
    // Construct message (set ~W bit low, MB bit low)
    msg[0] = 0x00 | reg;
    msg[1] = data;

    // Write to register
    gpio_put(cs, 0);
    spi_write_blocking(spi, msg, 2);
    gpio_put(cs, 1);
}

int spi_reg_read(spi_inst_t *spi,
                const uint cs,
                const uint8_t reg,
                uint8_t *buf,
                uint8_t nbytes) {
    int num_bytes_read = 0;
    uint8_t mb = 0;

    // Determine if multiple byte (MB) bit should be set
    if (nbytes < 1) {
        return -1;
    } else if (nbytes == 1) {
        mb = 0;
    } else {
        mb = 1;
    }

    // Construct message (set ~W bit high)
    uint8_t msg = 0x80 | (mb << 6) | reg;

    // Read from register
    gpio_put(cs, 0);
    spi_write_blocking(spi, &msg, 1);
    num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
    gpio_put(cs, 1);

    return num_bytes_read;
}