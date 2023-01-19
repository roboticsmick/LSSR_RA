/*
 * LSR_MS56xx.c
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
#include "LSR_MS56xx.h"
#include "LSR_serial.h"

bool MS56xx_Init(i2c_inst_t *i2c) {

    uint8_t i;

    // Buffer to store raw reads
    uint8_t data[6];

    // MS5611 reset command
    data[0] = 0x00;
    i2c_reg_write(i2c, MS56xx_ADDR, MS56xx_RESET_COMMAND, &data[0], 1);

    // Wait before reading PROM measurements
    sleep_ms(2000);
    for( i=0 ; i< MS5611_COEFFICIENT_NUMBERS ; i++)
	{
        i2c_reg_read(i2c, MS56xx_ADDR, MS56xx_PROM_ADDRESS_READ_ADDRESS_0 + i*2, data, 2);
        eeprom_coeff[i] = (data[0] << 8) | data[1];
	}

    return true;
}



bool MS56xx_Avg(i2c_inst_t *i2c, MS56xx_data_t *MS56xx_data) {

    return true;
}


bool MS56xx_Read(i2c_inst_t *i2c, MS56xx_data_t *MS56xx_data) {

    uint8_t i;
    uint32_t adc_temperature, adc_pressure;
    int32_t dT, temp, press, alt, temp_out;
    int64_t OFF, SENS, P, T2, OFF2, SENS2;

    // Buffer to store raw reads
    uint8_t data[6];

    // start conversion of the pressure sensor
    data[0] = 0x00;
    i2c_reg_write(i2c, MS56xx_ADDR, 0x44, &data[0], 1); // Pressure resolution RMS 0x44 = 1024 
    // Delay while conversion 
    sleep_ms(2);
    // read the pressure
    i2c_reg_read(i2c, MS56xx_ADDR, 0x00, data, 3);
    sleep_ms(2);
    // extract the raw value
    adc_pressure  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        
    sleep_ms(10);
    // start conversion of the pressure sensor
    data[0] = 0x00;
    i2c_reg_write(i2c, MS56xx_ADDR, 0x54, &data[0], 1); // Temperature resolution RMS 0x54 = 1024 
    // Delay while conversion 
    sleep_ms(2);
    // read the temperature
    i2c_reg_read(i2c, MS56xx_ADDR, 0x00, data, 3);
    // extract the raw value
    sleep_ms(2);
    adc_temperature  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    // convert the pressure reading

    dT = (int32_t)adc_temperature - ((int32_t)eeprom_coeff[MS5611_REFERENCE_TEMPERATURE_INDEX] <<8 );
    temp = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

    // Second order temperature compensation if temp below 20 degrees
    if( temp < 2000 )
    {
        T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
        OFF2 = 61 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000) / 16 ;
        SENS2 = 29 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000) / 16 ;
        
        // Temperature compensation if temp below -15 degrees
        if( temp < -1500 )
        {
            OFF2 += 17 * ((int64_t)temp + 1500) * ((int64_t)temp + 1500) ;
            SENS2 += 9 * ((int64_t)temp + 1500) * ((int64_t)temp + 1500) ;
        }
    }
    else
    {
        T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
        OFF2 = 0 ;
        SENS2 = 0 ;
    }

    // OFF = OFF_T1 + TCO * dT
    OFF = ( (int64_t)(eeprom_coeff[MS5611_PRESSURE_OFFSET_INDEX]) << 16 ) + ( ( (int64_t)(eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 7 ) ;
    OFF -= OFF2 ;

    // Sensitivity at actual temperature = SENS_T1 + TCS * dT
    SENS = ( (int64_t)eeprom_coeff[MS5611_PRESSURE_SENSITIVITY_INDEX] << 15 ) + ( ((int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 8 ) ;
    SENS -= SENS2 ;

    press = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15;

    MS56xx_data->baro_temp_float = ((float)temp - T2 ) / 100;
    MS56xx_data->pressure_float = (float)press / 100;
    MS56xx_data->alt_float = ((((pow((MS56xx_data->sea_level_pressure/MS56xx_data->pressure_float ), INV_GAMMA)) - 1) * (MS56xx_data->baro_temp_float + CONVERT_C_TO_K))/TEMP_LAPSE_RATE);

    return true;
}



/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */

