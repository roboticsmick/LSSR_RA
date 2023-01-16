#include <stdio.h>
#include <math.h>
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

// MS5611 EEPROM address
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_0 = 0xA0;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_1 = 0xA2;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_2 = 0xA4;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_3 = 0xA6;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_4 = 0xA8;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_5 = 0xAA;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_6 = 0xAC;
static const uint8_t MS5611_PROM_ADDRESS_READ_ADDRESS_7 = 0xAE;

// Coefficients indexes for temperature and pressure computation
static const uint8_t MS5611_PRESSURE_SENSITIVITY_INDEX = 1;
static const uint8_t MS5611_PRESSURE_OFFSET_INDEX = 2;
static const uint8_t MS5611_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX = 3;
static const uint8_t MS5611_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX = 4;
static const uint8_t MS5611_REFERENCE_TEMPERATURE_INDEX = 5;
static const uint8_t MS5611_TEMP_COEFF_OF_TEMPERATURE_INDEX = 6;
static const uint8_t MS5611_CRC_INDEX = 7;
static const uint8_t MS5611_COEFFICIENT_NUMBERS = 8;

static uint16_t eeprom_coeff[MS5611_COEFFICIENT_NUMBERS];

// Other constants
/* write to one of these addresses to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR256  0x40
#define ADDR_CMD_CONVERT_D1_OSR512  0x42
#define ADDR_CMD_CONVERT_D1_OSR1024 0x44
#define ADDR_CMD_CONVERT_D1_OSR2048 0x46
#define ADDR_CMD_CONVERT_D1_OSR4096 0x48

/* write to one of these addresses to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR256  0x50
#define ADDR_CMD_CONVERT_D2_OSR512  0x52
#define ADDR_CMD_CONVERT_D2_OSR1024 0x54
#define ADDR_CMD_CONVERT_D2_OSR2048 0x56
#define ADDR_CMD_CONVERT_D2_OSR4096 0x58

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
 */
static const uint8_t ADDR_CMD_CONVERT_PRESSURE = ADDR_CMD_CONVERT_D1_OSR1024;
static const uint8_t ADDR_CMD_CONVERT_TEMPERATURE = ADDR_CMD_CONVERT_D2_OSR1024;

#define ALTITUDE6_CONVERT_Pa_TO_mbar                  100
#define ALTITUDE6_TEMP_CONVERT_C_TO_K                 273.15
#define ALTITUDE6_ISA_MODEL_PARAM                       0.1902225603956629
#define ALTITUDE6_CONVERT_NEG                           1
#define ALTITUDE6_STANDARD_TEMPERATURE_LAPSE_RATE       0.0065
#define ALTITUDE6_TEMPERATURE_COEFF                  2000
#define ALTITUDE6_7_BIT_DATA                          128
#define ALTITUDE6_8_BIT_DATA                          256
#define ALTITUDE6_15_BIT_DATA                       32768
#define ALTITUDE6_16_BIT_DATA                       65536
#define ALTITUDE6_21_BIT_DATA                     2097152
#define ALTITUDE6_23_BIT_DATA                     8388608
#define ALTITUDE6_TEMPERATURE_CONV_TO_C               100
#define ALTITUDE6_STANDARD_ATMOSPHERE_mbar           1013.25


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
    uint8_t i;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    float acc_x_f;
    float acc_y_f;
    float acc_z_f;
    float pressure_float;
    float baro_temp_float;
    float alt_float;

    float SEA_LEVEL_PRESSURE = 1018.6; // Brisbane pressure at sea level (MSL) = 1018.6 hPa * 1000
    const int32_t TEMP_LAPSE_RATE = 6500; // temperature lapse rate (0.0065*10000)
    const int32_t GAS_CONSTANT = 831447; // Universal gas constant (8.31447*10000)
    const int32_t MOLAR_MASS_AIR = 2897; // molar mass of air (0.02897*10000)
    const int32_t GRAVITY = 980665; // acceleration due to gravity (9.80665*10000)
    const int32_t SCALE_FACTOR = 10000;

    uint32_t adc_temperature, adc_pressure;
    int32_t dT, temp, press, alt, temp_out;
    int64_t OFF, SENS, P, T2, OFF2, SENS2;

    // float D1, D2; // Press, Temp
    // float dT;
    // float TEMP;
    // float OFF;
    // float SENS;
    
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

    // Wait before reading PROM measurements
    sleep_ms(2000);
    for( i=0 ; i< MS5611_COEFFICIENT_NUMBERS ; i++)
	{
        reg_read(i2c, MS56xx_ADDR, MS5611_PROM_ADDRESS_READ_ADDRESS_0 + i*2, data, 2);
        eeprom_coeff[i] = (data[0] << 8) | data[1];
	}

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
        
        // for( i=1 ; i< MS5611_COEFFICIENT_NUMBERS-2; i++)
        //     {
        //         printf("%i. %d\r\n", i, eeprom_coeff[MS5611_PRESSURE_SENSITIVITY_INDEX]);
        //     }

        // start conversion of the pressure sensor
        data[0] = 0x00;
        reg_write(i2c, MS56xx_ADDR, 0x44, &data[0], 1); // Pressure resolution RMS 0x44 = 1024 
        // Delay while conversion 
        sleep_ms(2);
		// read the pressure
		reg_read(i2c, MS56xx_ADDR, 0x00, data, 3);
        sleep_ms(2);
        // extract the raw value
		adc_pressure  = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
            
        sleep_ms(10);
        // start conversion of the pressure sensor
        data[0] = 0x00;
        reg_write(i2c, MS56xx_ADDR, 0x54, &data[0], 1); // Temperature resolution RMS 0x54 = 1024 
        // Delay while conversion 
        sleep_ms(2);
        // read the temperature
		reg_read(i2c, MS56xx_ADDR, 0x00, data, 3);
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

        press = ( ( (adc_pressure * SENS) >> 21 ) - OFF ) >> 15 ;
        baro_temp_float = ((float)temp - T2 ) / 100;
        pressure_float = (float)press / 100;

        // h = (1 - (P/P0)^(1/5.257)) * T/0.0065
        // h = altitude (m)
        // P = air pressure at altitude (hPa)
        // P0 = air pressure at sea level (hPa)
        // T = temperature at altitude (°C)
        alt_float = (1 - pow((pressure_float/SEA_LEVEL_PRESSURE), (1/5.257))) * baro_temp_float/0.0065;

        // Print results
        printf("Pressure: %.2f | Temperature: %.2f | Altitude :%.2f\r\n", pressure_float, baro_temp_float, alt_float);

        gpio_put(led_pin, true);
        sleep_ms(200);
    }
}