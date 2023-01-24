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
        ms5611->ms5611_data_ready = MS5611_READY;
    }

    // Print EEPROM Values
    // for( i=0 ; i< MS5611_COEFF_NUMBERS ; i++)
    // {
    //     printf("%i.: %li\r\n",i,eeprom_coeff[i]);
    // }

    return true;
}




//bool MS5611_Read(i2c_inst_t *i2c, MS5611_data_t *MS5611_data) {

bool ms5611_adc_start(ms5611_data_t *ms5611) {

    // Buffer to store EEPROM values
    uint8_t data[1];

    if (ms5611->ms5611_sensor_read == READ_TEMPERATURE) {
        data[0] = 0x00;
        i2c_reg_write(ms5611->i2c_address, MS5611_ADDR, MS5611_RESET_COMMAND, &data[0], 1);
    }

    // }
    // // Buffer to store raw reads
    // uint8_t data[6];
    // // start conversion of the pressure sensor
    // data[0] = 0x00;
    // i2c_reg_write(MS5611_data->i2c_ptr, MS5611_ADDR, 0x44, &data[0], 1); // Pressure resolution RMS 0x44 = 1024 
    // // read the pressure
    // i2c_reg_read(MS5611_data->i2c_ptr, MS5611_ADDR, 0x00, data, 3);

    // // extract the raw value
    // adc_pressure  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    
    // sleep_ms(10);
    // // start conversion of the Temperature sensor
    // data[0] = 0x00;
    // i2c_reg_write(MS5611_data.i2c_ptr, MS5611_ADDR, 0x54, &data[0], 1); // Temperature resolution RMS 0x54 = 1024 
    // // Delay while conversion 
    // sleep_ms(2);
    // // read the temperature
    // i2c_reg_read(MS5611_data.i2c_ptr, MS5611_ADDR, 0x00, data, 3);
    // // extract the raw value
    // sleep_ms(2);
    // adc_temperature  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    // // convert the pressure reading

    // dT = (int32_t)adc_temperature - ((int32_t)eeprom_coeff[MS5611_REFERENCE_TEMPERATURE_INDEX] <<8 );
    // temp = 2000 + ((int64_t)dT * (int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23) ;

    // // Second order temperature compensation if temp below 20 degrees
    // if( temp < 2000 )
    // {
    //     T2 = ( 3 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 33;
    //     OFF2 = 61 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000) / 16 ;
    //     SENS2 = 29 * ((int64_t)temp - 2000) * ((int64_t)temp - 2000) / 16 ;
        
    //     // Temperature compensation if temp below -15 degrees
    //     if( temp < -1500 )
    //     {
    //         OFF2 += 17 * ((int64_t)temp + 1500) * ((int64_t)temp + 1500) ;
    //         SENS2 += 9 * ((int64_t)temp + 1500) * ((int64_t)temp + 1500) ;
    //     }
    // }
    // else
    // {
    //     T2 = ( 5 * ( (int64_t)dT  * (int64_t)dT  ) ) >> 38;
    //     OFF2 = 0 ;
    //     SENS2 = 0 ;
    // }

    // // OFF = OFF_T1 + TCO * dT
    // OFF = ( (int64_t)(eeprom_coeff[MS5611_PRESSURE_OFFSET_INDEX]) << 16 ) + ( ( (int64_t)(eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT ) >> 7 ) ;
    // OFF -= OFF2 ;

    // // Sensitivity at actual temperature = SENS_T1 + TCS * dT
    // SENS = ( (int64_t)eeprom_coeff[MS5611_PRESSURE_SENSITIVITY_INDEX] << 15 ) + ( ((int64_t)eeprom_coeff[MS5611_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 8 ) ;
    // SENS -= SENS2 ;

    // press = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15;
    // MS5611_data.baro_temp_float = ((float)temp - T2 ) / 100;
    // MS5611_data.pressure_float = (float)press / 100;
    // MS5611_data.alt_float = ((((pow((MS5611_data->sea_level_pressure/MS5611_data->pressure_float ), INV_GAMMA)) - 1) * (MS5611_data->baro_temp_float + CONVERT_C_TO_K))/TEMP_LAPSE_RATE);
    // printf("Function Pressure: %.2f | Temperature: %.2f | Altitude :%.2f\r\n", MS5611_data->pressure_float, MS5611_data->baro_temp_float, MS5611_data->alt_float);

    return true;
}



/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */


bool ms5611_crc_check (uint16_t *n_prom, uint8_t crc)
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

