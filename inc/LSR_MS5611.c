/*
 * LSR_MS5611.c
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
#include "LSR_MS5611.h"
#include "LSR_serial.h"

bool ms5611_i2c_init(ms5611_data_t *ms5611, i2c_inst_t *i2c, float *sea_level_pressure) {
    // Initialise variables
    uint8_t i;
    // Buffer to store EEPROM values
    uint8_t data[6];
    // Set I2C
    ms5611->i2c_address = i2c;
    // Set sea level pressure 
    ms5611->sea_level_pressure = *sea_level_pressure;
    // MS5611 reset command
    data[0] = 0x00;
    i2c_reg_write(ms5611->i2c_address, MS5611_ADDR, MS5611_RESET_COMMAND, &data[0], 1);
    // Wait before reading EEPROM measurements
    sleep_ms(2000);
    for( i=0 ; i< MS5611_COEFF_NUMBERS; i++)
    {
        i2c_reg_read(ms5611->i2c_address, MS5611_ADDR, MS5611_PROM_ADDRESS_READ_0 + i*2, data, 2);
        eeprom_coeff[i] = (data[0] << 8) | data[1];
    }
    if(!ms5611_crc_check( eeprom_coeff, eeprom_coeff[MS5611_CRC_INDEX] & 0x000F )){
        ms5611->ms5611_coeff_check = COEFF_ERROR;
        return false;
    }
    else {
        ms5611->ms5611_coeff_check = COEFF_VALID;
        ms5611->ms5611_sensor_read = READ_TEMPERATURE;
        ms5611->ms5611_sensor_ready = MS5611_READY;
        ms5611->ms5611_adc_ready = MS5611_ADC_PENDING;
    }

    return true;
}


int64_t alarm_callback(int32_t alarm_id_t, void *user_data) {
    ms5611_data_t *ms5611 = (ms5611_data_t *)user_data;
    // ms5611->sea_level_pressure++;
    ms5611->ms5611_adc_ready = MS5611_ADC_READY;
    return 0;
}


bool ms5611_adc_start(ms5611_data_t *ms5611) {
    // Start ADC sensor read
    uint8_t data[1];
    data[0] = 0x00;
    if (ms5611->ms5611_sensor_read == READ_TEMPERATURE) {
        i2c_reg_write(ms5611->i2c_address, MS5611_ADDR, MS5611_ADDR_CMD_D1_OSR1024, &data[0], 1);
        ms5611->ms5611_sensor_ready = MS5611_PENDING;
    } 
    else {
        i2c_reg_write(ms5611->i2c_address, MS5611_ADDR, MS5611_ADDR_CMD_D2_OSR1024, &data[0], 1);
        ms5611->ms5611_sensor_ready = MS5611_PENDING;
    }
    add_alarm_in_ms(MS5611_CONVERSION_TIME_OSR_1024, alarm_callback, ms5611, false);
    return true;
}


bool ms5611_adc_read(ms5611_data_t *ms5611) {
    
    // Set data buffer vatiable
    uint8_t data[3];
	data[0] = 0;
	data[1] = 0;
	data[2] = 0;
    // Read MS5611 ADC
    i2c_reg_read(ms5611->i2c_address, MS5611_ADDR, MS5611_READ_ADC, data, 3);

    if (ms5611->ms5611_sensor_read == READ_TEMPERATURE) {
        // Store ADC temp value as uint32_t
        ms5611->temp_adc  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        ms5611->ms5611_sensor_read = READ_PRESSURE;
    } 
    else {
        // Store ADC pressure value as uint32_t
        ms5611->pressure_adc  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
        // Calculate pressure, temp, and altitude
        ms5611_calc(ms5611);
        // Set state to read temperature ADC
        ms5611->ms5611_sensor_read = READ_TEMPERATURE;
    }
    // Set sensor state to ready
    ms5611->ms5611_sensor_ready = MS5611_READY;
    // Set sensor ADC state to pending
    ms5611->ms5611_adc_ready = MS5611_ADC_PENDING;
    return true;
}


bool ms5611_calc(ms5611_data_t *ms5611) {
    // Initialise variables
	int32_t dT, TEMP;
	int64_t OFF, SENS, PRESS, T2, OFF2, SENS2;

    // Left shift << 8 multiplies the value by 2^8 (256 in decimal).
    dT = (int32_t)ms5611->temp_adc - ((int32_t)eeprom_coeff[MS5611_REF_TEMP_INDEX] <<8 );
    
    // Right shift >> 23 divides the value by 2^23 (8388608 in decimal)
    TEMP = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_TEMP_INDEX] >> 23) ;

    // Second order temperature compensation if temp below 20 degrees
    if( TEMP < 2000 )
    {
        // Right shift >> 33 divides the value by 2^33 (8589934592 in decimal)
        T2 = ( 3 * ( (int64_t)dT * (int64_t)dT  ) ) >> 33;
        OFF2 = 61 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
        SENS2 = 29 * ((int64_t)TEMP - 2000) * ((int64_t)TEMP - 2000) / 16 ;
        
        // Temperature compensation if temp below -15 degrees
        if( TEMP < -1500 )
        {
            OFF2 += 17 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
            SENS2 += 9 * ((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500) ;
        }
    }
    else
    {
        // Right shift >> 38 divides the value by 2^38 (274877906944 in decimal)
        T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
        OFF2 = 0 ;
        SENS2 = 0 ;
    }

    // OFF = OFF_T1 + TCO * dT
    // Left shift << 16 multiplies the value by 2^16 (65536 in decimal).
    OFF = ( (int64_t)(eeprom_coeff[MS5611_PRESS_OFFSET_INDEX]) << 16 ) + ( ( (int64_t)(eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESS_OFF_INDEX]) * dT ) >> 7 ) ;
    OFF -= OFF2 ;

    // Sensitivity at actual temperature = SENS_T1 + TCS * dT
    // Left shift << 15 multiplies the value by 2^15 (32768 in decimal).
    SENS = ( (int64_t)eeprom_coeff[MS5611_PRESS_SENS_INDEX] << 15 ) + ( ((int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESS_SEN_INDEX] * dT) >> 8 ) ;
    SENS -= SENS2 ;
    
    // Right shift >> 21 divides the value by 2^21 (xxx in decimal)
    // Right shift >> 15 divides the value by 2^15 (32768 in decimal)
    PRESS = ( ( (ms5611->pressure_adc * SENS) >> 21 ) - OFF ) >> 15;
    ms5611->baro_temp_float = ((float)TEMP - T2 ) / 100;
    ms5611->pressure_float = (float)PRESS / 100;
    ms5611->alt_float = ((((pow((ms5611->sea_level_pressure/ms5611->pressure_float ), INV_GAMMA)) - 1) * (ms5611->baro_temp_float + CONVERT_C_TO_K))/TEMP_LAPSE_RATE);
    return true;
}


bool ms5611_filter(ms5611_data_t *ms5611) {
    return true;
}


/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */


bool ms5611_crc_check(uint16_t *n_prom, uint8_t crc)
{
    uint8_t cnt, n_bit; 
    uint16_t n_rem; 
    uint16_t crc_read;
    n_rem = 0x00;
    crc_read = n_prom[7]; 
    n_prom[7] = (0xFF00 & (n_prom[7])); 
    for (cnt = 0; cnt < 16; cnt++) 
    {
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000))
                n_rem = (n_rem << 1) ^ 0x3000;
            else
                n_rem = (n_rem << 1);
        }
    }
    n_rem = (0x000F & (n_rem >> 12)); 
    n_prom[7] = crc_read;
    n_rem ^= 0x00;
    return  ( n_rem == crc );
}


bool print_eeprom(void) {
    // Initialise variables
    uint8_t i;
    // Print EEPROM Values
    for( i=0 ; i< MS5611_COEFF_NUMBERS ; i++)
    {
        printf("%i.: %li\r\n",i,eeprom_coeff[i]);
    }
}