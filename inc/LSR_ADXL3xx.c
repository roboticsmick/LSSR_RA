/*
 * LSR_ADXL3xxx.c
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      https://www.analog.com/media/en/technical-documentation/data-sheets/adxl375.pdf 
 * 
 * ADXL3xx_data_t ADXL3xx_data = {0,0,0,0,0,0};
 * ADXL3xxx_Init(i2c0_ptr);
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "LSR_ADXL3xx.h"
#include "LSR_serial.h"

bool ADXL3xxx_Init(i2c_inst_t *i2c) {

    uint8_t data[1];

    // Read device ID to make sure that we can communicate with the ADXL343
    i2c_reg_read(i2c, ADXL343_ADDR, REG_DEVID, data, 1);
    if (data[0] != DEVID) {
        printf("ERROR: Could not communicate with ADXL343\r\n");
        return false;
    }

    // Read Power Control register
    i2c_reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL, data, 1);
    printf("0x%X\r\n", data[0]);

    // Tell ADXL343 to start taking measurements by setting Measure bit to high
    data[0] |= (1 << 3);
    i2c_reg_write(i2c, ADXL343_ADDR, REG_POWER_CTL, &data[0], 1);

    // Test: read Power Control register back to make sure Measure bit was set
    i2c_reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL, data, 1);
    printf("0x%X\r\n", data[0]);

    // Wait before taking measurements
    sleep_ms(2000);
    return true;
} 

bool ADXL3xxx_Read(i2c_inst_t *i2c, ADXL3xx_data_t *ADXL3xx_data) {

    // Buffer to store raw reads
    uint8_t data[6];

    // Read X, Y, and Z values from registers (16 bits each)
    i2c_reg_read(i2c, ADXL343_ADDR, REG_DATAX0, data, 6);

    // Convert 2 bytes (little-endian) into 16-bit integer (signed)
    ADXL3xx_data->acc_x = (int16_t)((data[1] << 8) | data[0]);
    ADXL3xx_data->acc_y = (int16_t)((data[3] << 8) | data[2]);
    ADXL3xx_data->acc_z = (int16_t)((data[5] << 8) | data[4]);

    // Convert measurements to [m/s^2]
    ADXL3xx_data->acc_x_f = ADXL3xx_data->acc_x * SENSITIVITY_2G * EARTH_GRAVITY;
    ADXL3xx_data->acc_y_f = ADXL3xx_data->acc_y * SENSITIVITY_2G * EARTH_GRAVITY;
    ADXL3xx_data->acc_z_f = ADXL3xx_data->acc_z * SENSITIVITY_2G * EARTH_GRAVITY;

    // Print results
    printf("X: %.2f | Y: %.2f | Z: %.2f\r\n", ADXL3xx_data->acc_x_f, ADXL3xx_data->acc_y_f, ADXL3xx_data->acc_z_f);
} 