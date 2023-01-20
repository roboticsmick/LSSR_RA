#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "../inc/LSR_MS56xx.h"

// ADXL3xx I2C address
static const uint8_t ADXL343_ADDR = 0x53;

// ADXL3xx Commands
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;
static const uint8_t ADXL343_DEVID = 0xE5;

// Other constants
static const float SENSITIVITY_2G = 1.0 / 256;  // (g/LSB)
static const float EARTH_GRAVITY = 9.80665;     // Earth's gravity in [m/s^2]

/*******************************************************************************
 * Function Declarations
 */
// int reg_write(i2c_inst_t *i2c, 
//                 const uint addr, 
//                 const uint8_t reg, 
//                 uint8_t *buf,
//                 const uint8_t nbytes);

// int reg_read(   i2c_inst_t *i2c,
//                 const uint addr,
//                 const uint8_t reg,
//                 uint8_t *buf,
//                 const uint8_t nbytes);

// /*******************************************************************************
//  * Function Definitions
//  */

// // Write 1 byte to the specified register
// int reg_write(  i2c_inst_t *i2c, 
//                 const uint addr, 
//                 const uint8_t reg, 
//                 uint8_t *buf,
//                 const uint8_t nbytes) {

//     int num_bytes_read = 0;
//     uint8_t msg[nbytes + 1];

//     // Check to make sure caller is sending 1 or more bytes
//     if (nbytes < 1) {
//         return 0;
//     }

//     // Append register address to front of data packet
//     msg[0] = reg;
//     for (int i = 0; i < nbytes; i++) {
//         msg[i + 1] = buf[i];
//     }

//     // Write data to register(s) over I2C
//     i2c_write_blocking(i2c, addr, msg, (nbytes + 1), false);

//     return num_bytes_read;
// }

// // Read byte(s) from specified register. If nbytes > 1, read from consecutive
// // registers.
// int reg_read(  i2c_inst_t *i2c,
//                 const uint addr,
//                 const uint8_t reg,
//                 uint8_t *buf,
//                 const uint8_t nbytes) {

//     int num_bytes_read = 0;

//     // Check to make sure caller is asking for 1 or more bytes
//     if (nbytes < 1) {
//         return 0;
//     }

//     // Read data from register(s) over I2C
//     i2c_write_blocking(i2c, addr, &reg, 1, true);
//     num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

//     return num_bytes_read;
// }

/*******************************************************************************
 * Main
 */
int main() {
    uint8_t i;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
    MS56xx_data_t *MS5611_data;
    
    MS5611_data->sea_level_pressure = 1008.6;

    // Pins
    const uint sda_pin = 16;
    const uint scl_pin = 17;
    const uint led_pin = 25;

    // Ports
    i2c_inst_t *i2c = i2c0;

    // Buffer to store raw reads
    uint8_t data[6];

    // Initialize chosen serial port
    stdio_init_all();

    //Initialize I2C port at 400 kHz
    i2c_init(i2c, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);

    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);
    gpio_put(led_pin, true);

    
    // Read device ID to make sure that we can communicate with the ADXL343
    // reg_read(i2c, ADXL343_ADDR, REG_DEVID, data, 1);
    // if (data[0] != DEVID) {
    //     printf("ERROR: Could not communicate with ADXL343\r\n");
    //     while (true);
    // }

    // Read Power Control register
    // reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL, data, 1);
    // printf("0x%X\r\n", data[0]);

    // Tell ADXL343 to start taking measurements by setting Measure bit to high
    // data[0] |= (1 << 3);
    // reg_write(i2c, ADXL343_ADDR, REG_POWER_CTL, &data[0], 1);

    // Test: read Power Control register back to make sure Measure bit was set
    // reg_read(i2c, ADXL343_ADDR, REG_POWER_CTL, data, 1);
    // printf("0x%X\r\n", data[0]);

    // // MS5611 reset command
    // data[0] = 0x00;
    // reg_write(i2c, MS56xx_ADDR, MS56xx_RESET_COMMAND, &data[0], 1);

    // // Wait before reading PROM measurements
    // sleep_ms(2000);
    // for( i=0 ; i< MS5611_COEFFICIENT_NUMBERS ; i++)
	// {
    //     reg_read(i2c, MS56xx_ADDR, MS5611_PROM_ADDRESS_READ_ADDRESS_0 + i*2, data, 2);
    //     eeprom_coeff[i] = (data[0] << 8) | data[1];
	// }


    MS56xx_Init(i2c);

    // Loop forever
    while (true) {

        gpio_put(led_pin, false);

        // // Read X, Y, and Z values from registers (16 bits each)
        // reg_read(i2c, ADXL343_ADDR, REG_DATAX0, data, 6);

        // // Convert 2 bytes (little-endian) into 16-bit integer (signed)
        // acc_x = (int16_t)((data[1] << 8) | data[0]);
        // acc_y = (int16_t)((data[3] << 8) | data[2]);
        // acc_z = (int16_t)((data[5] << 8) | data[4]);

        // // Convert measurements to [m/s^2]
        // acc_x_f = acc_x * SENSITIVITY_2G * EARTH_GRAVITY;
        // acc_y_f = acc_y * SENSITIVITY_2G * EARTH_GRAVITY;
        // acc_z_f = acc_z * SENSITIVITY_2G * EARTH_GRAVITY;

        // Print results
        // printf("X: %.2f | Y: %.2f | Z: %.2f\r\n", acc_x_f, acc_y_f, acc_z_f);
        
        // for( i=1 ; i< MS5611_COEFFICIENT_NUMBERS-2; i++)
        //     {
        //         printf("%i. %d\r\n", i, eeprom_coeff[MS5611_PRESSURE_SENSITIVITY_INDEX]);
        //     }

        MS56xx_Read(i2c, MS5611_data);
        printf("Main Pressure: %.2f | Temperature: %.2f | Altitude :%.2f\r\n", MS5611_data->pressure_float, MS5611_data->baro_temp_float, MS5611_data->alt_float);

        // Print results

        gpio_put(led_pin, true);
        sleep_ms(2000);
    }
}