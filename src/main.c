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
    float sea_level_pressure = 1013.5;

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
        printf("sen: %i\r\n", ms5611.ms5611_sensor_ready);
        printf("adc: %i\r\n", ms5611.ms5611_adc_ready);
        if (ms5611.ms5611_sensor_ready == MS5611_READY) {
            printf("2");
            ms5611_adc_start(&ms5611); 
        }
        sleep_ms(1000);
        if (ms5611.ms5611_adc_ready == MS5611_ADC_READY) {
            printf("3");
            ms5611_adc_read(&ms5611); 
        }
        sleep_ms(1000);
        // // sleep_ms(4000);
        // //gpio_put(led_pin, false);
        // //printf("Main: Sea level pressure: %.2f\r\n", MS5611_data.sea_level_pressure);
        // //printf("Main Pressure: %.2f | Temperature: %.2f | Altitude :%.2f\r\n", MS5611_data->pressure_float, MS5611_data->baro_temp_float, MS5611_data->alt_float);
        // printf("Sea Pressure: %.2f\r\n", ms5611.sea_level_pressure);
        // //gpio_put(led_pin, true);
        // sleep_ms(4000);
    }
}

// bool ms5611_timer_func(struct repeating_timer *ms5611_timer) {
//     // Read MS5611 ADC
//     ms5611_data_t *ms5611 = (ms5611_data_t*)ms5611_timer->user_data;
//     // MS5611_Read();
//     // ms5611->sea_level_pressure++;
//     return true;
// }


// int64_t alarm_callback(alarm_id_t, void *user_data) {
//     ms5611_data_t *ms5611 = (ms5611_data_t*)user_data;
//     ms5611->data_ready = MS5611_READY;
//     return 0;
// }



    // ms5611.i2c_address = i2c;

    // // Create interrupt timer for MS5611
    // struct repeating_timer ms5611_timer;
    
    // // Create delay in ms with callback function, user data, and timer struct
    // add_repeating_timer_ms(2000, ms5611_timer_func, &ms5611, &ms5611_timer);
