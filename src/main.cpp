#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "../inc/LSR_MS5611.h"


// Initialise MS5611 
// bool MS5611_timer_func(struct repeating_timer *MS5611_timer) {
//     // Read MS5611 ADC
//     MS5611_Read();
//     return true;
// }

/*******************************************************************************
 * Main
 */
int main() {

    // Initialize MS5611 barometer sensor
    static ms5611_data_t ms5611;

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

    //MS5611_data_t MS5611_data;
    ms5611.sea_level_pressure = 1013.5;
    ms5611.i2c_address = i2c;
    // Initialise MS5611 by resetting and storing EEPROM values
    ms5611_init(&ms5611);

    // Create interrupt timer for MS5611
    //struct repeating_timer MS5611_timer;
    
    // Create delay in ms with callback function, user data, and timer struct
    //add_repeating_timer_ms(2000, MS5611_timer_func, NULL, &MS5611_timer);

    // Loop forever
    while (true) {
        // 
        sleep_ms(4000);
        gpio_put(led_pin, false);
        //printf("Main: Sea level pressure: %.2f\r\n", MS5611_data.sea_level_pressure);
        //MS5611_Read(i2c, MS5611_data);
        //printf("Main Pressure: %.2f | Temperature: %.2f | Altitude :%.2f\r\n", MS5611_data->pressure_float, MS5611_data->baro_temp_float, MS5611_data->alt_float);
        gpio_put(led_pin, true);
        sleep_ms(4000);
    }
}