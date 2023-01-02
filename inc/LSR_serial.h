/*
 * LSR_serial.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 * 
 *      Pin variable names:
 *      CS/CSB
 *      TX/MOSI/SDI/SDA
 *      RX/MISO/SDO
 *      SCLK/SCK/SCL
 */

#ifndef _LSR_SERIAL_H
#define _LSR_SERIAL_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C constants
static const uint I2C_FREQ_400KHZ = 400;

// Serial Constants
static const uint SPI_FREQ_1MHZ = 1000;
static const uint SPI_POLARITY_1 = 1;
static const uint SPI_PHASE_1 = 1;
static const uint SPI_BITS_PER_TRANSFER_8 = 8;


bool i2c_setup(i2c_inst_t *i2c,
                const uint sda_pin, 
                const uint scl_pin);

int i2c_reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int i2c_reg_read(i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

bool spi_setup(spi_inst_t *spi,
                const uint cs_pin,
                const uint sck_pin,
                const uint sdi_pin, 
                const uint sdo_pin,
                const uint spi_freq);
    
bool spi_cs_setup(const uint cs_pin);

void spi_reg_write(spi_inst_t *spi, 
                const uint cs, 
                const uint8_t reg, 
                const uint8_t data);

int spi_reg_read(spi_inst_t *spi,
                const uint cs,
                const uint8_t reg,
                uint8_t *buf,
                uint8_t nbytes);

#ifdef __cplusplus
}
#endif
#endif