#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


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

// MS5611 I2C address
static const uint8_t MS56xx_ADDR = 0x77;

// MS5611 device commands
static const uint8_t MS56xx_RESET_COMMAND = 0x1E;
static const uint8_t MS56xx_START_PRESSURE_ADC_CONVERSION = 0x40;
static const uint8_t MS56xx_START_TEMPERATURE_ADC_CONVERSION = 0x50;
static const uint8_t MS56xx_READ_ADC = 0x00;
static const uint8_t MS56xx_CONVERSION_OSR_MASK = 0x0F;

static const uint8_t MS5611_CONVERSION_TIME_OSR_256 = 1;
static const uint8_t MS5611_CONVERSION_TIME_OSR_512 = 2;
static const uint8_t MS5611_CONVERSION_TIME_OSR_1024 = 3;
static const uint8_t MS5611_CONVERSION_TIME_OSR_2048 = 5;
static const uint8_t MS5611_CONVERSION_TIME_OSR_4096 = 9;

#define PROM_SENS       0
#define PROM_OFF        1
#define PROM_TCS        2
#define PROM_TCO        3
#define PROM_TREF       4
#define PROM_TEMPSENS   5

/*******************************************************************************
 * Function Declarations
 */
int reg_write(i2c_inst_t *i2c, 
                const uint addr, 
                const uint8_t reg, 
                uint8_t *buf,
                const uint8_t nbytes);

int reg_read(   i2c_inst_t *i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

/*******************************************************************************
 * Function Definitions
 */

// Write 1 byte to the specified register
int reg_write(  i2c_inst_t *i2c, 
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
int reg_read(  i2c_inst_t *i2c,
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

/*******************************************************************************
 * Main
 */
int main() {

    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
    float pressure_float;
    float baro_temp_float;
    uint32_t adc_temperature, adc_pressure;
    int32_t dt, temp, press;
    int64_t off, sens, p, t2, off2, sens2;

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

    // MS5611 reset command
    data[0] = 0x00;
    reg_write(i2c, MS56xx_ADDR, MS56xx_RESET_COMMAND, &data[0], 1);
    printf("0x%X\r\n", data[0]);

    // MS5611 reset command
    data[0] = 0x00;
    reg_write(i2c, MS56xx_ADDR, MS56xx_RESET_COMMAND, &data[0], 1);
    printf("0x%X\r\n", data[0]);

    // Wait before taking measurements
    sleep_ms(2000);

    // Loop forever
    while (true) {

        gpio_put(led_pin, false);
        sleep_ms(1000);
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
        
        // start conversion of the pressure sensor
        data[0] = 0x00;
        reg_write(i2c, MS56xx_ADDR, 0x42, &data[0], 1); // Pressure resolution RMS 0x42= 512 
        // Delay while conversion 
        sleep_ms(10);
		// read the pressure
		reg_read(i2c, MS56xx_ADDR, 0x00, data, 3);
        // extract the raw value
		adc_pressure  = data[0] << 16 | data[1] << 8 | data[2];
        sleep_ms(10);
        // start conversion of the pressure sensor
        data[0] = 0x00;
        reg_write(i2c, MS56xx_ADDR, 0x52, &data[0], 1); // Temperature resolution RMS 0x52= 512 
        // Delay while conversion 
        sleep_ms(10);
        // read the temperature
		reg_read(i2c, MS56xx_ADDR, 0x00, data, 3);
        // extract the raw value
		adc_temperature  = data[0] << 16 | data[1] << 8 | data[2];
		// convert the pressure reading

        dt = (int32_t) adc_temperature - PROM_TREF * (1<<8);
        temp = (int32_t) 2000 + (int64_t) dt * PROM_TEMPSENS / (1<<23);

        t2 = ( 5 * ( (int64_t)dt  * (int64_t)dt ) ) >> 38;
		off2 = 0 ;
		sens2 = 0 ;

        off = (int64_t) PROM_OFF * (1<<16) + (int64_t) PROM_TCO * dt / (1<<7);
        sens = (int64_t) PROM_SENS * (1<<15) + (int64_t) PROM_TCS * dt / (1<<8);

        press = (uint32_t) ((adc_pressure * sens / (1<<21)) - off) / (1<<15);

        baro_temp_float = ( (float)temp - t2 ) / 100;
        pressure_float = (float)press / 100;

        // Print results
        printf("Pressure: %.2f | Temperature: %.2f\r\n", pressure_float, baro_temp_float);

        gpio_put(led_pin, true);
        sleep_ms(1000);
    }
}