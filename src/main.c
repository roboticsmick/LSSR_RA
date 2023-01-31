#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "../inc/LSR_MS5611.h"




/*******************************************************************************
 * Main
 */
int main() {

    // Variables
    float sea_level_pressure = 1000.4;

    // Pins
    const uint sda_pin = 16;
    const uint scl_pin = 17;
    const uint led_pin = 25;

    // Ports
    i2c_inst_t *i2c = i2c0;

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

    // Initialize MS5611 barometer sensor
    ms5611_data_t ms5611;

    // Initialise MS5611 by resetting and storing EEPROM values
    ms5611_i2c_init(&ms5611, i2c, &sea_level_pressure);

    // Loop forever
    while (true) {
        // MS5611 Sensor start ADC
        if (ms5611.ms5611_sensor_ready == MS5611_READY) {
            ms5611_adc_start(&ms5611); 
        }
        // MS5611 Sensor read ADC
        if (ms5611.ms5611_adc_ready == MS5611_ADC_READY) {
            ms5611_adc_read(&ms5611); 
        }
    }
}
