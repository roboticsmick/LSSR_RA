#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "../inc/LSR_MS56xx.h"

bool MS5611_timer_func(struct repeating_timer *MS5611_timer) {
    MS56xx_Read(i2c, &MS5611_data);
}

/*******************************************************************************
 * Main
 */
int main() {

    // Define class
    MS56xx_data_t MS5611_data;

    // Variables
    MS5611_data.sea_level_pressure = 1013.5;
    
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

    // Initialize MS5611 sensor
    MS56xx_Init(i2c);

    // Create interrupt timer for MS5611
    struct repeating_timer MS5611_timer;
    // Create delay in ms with callback function, user data, and timer struct
    add_repeating_timer_ms(MS56xx_CONVERSION_TIME_OSR_1024, MS5611_timer_func, NULL, &MS5611_timer);

    printf("Main: Sea level pressure: %.2f\r\n", MS5611_data.sea_level_pressure);

    // Loop forever
    while (true) {

        gpio_put(led_pin, false);

        MS56xx_Read(i2c, &MS5611_data);
        //printf("Main Pressure: %.2f | Temperature: %.2f | Altitude :%.2f\r\n", MS5611_data->pressure_float, MS5611_data->baro_temp_float, MS5611_data->alt_float);

        gpio_put(led_pin, true);
        sleep_ms(2000);
    }
}