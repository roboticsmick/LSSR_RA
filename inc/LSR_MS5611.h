/*
 * MS5611x.h
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      
 */

#ifndef _MS5611_H
#define _MS5611_H

#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// MS5611 I2C device address
#define MS5611_ADDR                             0x77
#define MS5611_ADDR_HIGH                        0x76

// MS5611 device commands
#define MS5611_RESET_COMMAND                    0x1E
#define MS5611_START_PRESSURE_ADC_CONVERSION    0x40
#define MS5611_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5611_READ_ADC                         0x00
#define MS5611_CONVERSION_OSR_MASK              0x0F

// MS5611 commands
#define MS5611_PROM_ADDRESS_READ_0      0xA0
#define MS5611_PROM_ADDRESS_READ_1      0xA2
#define MS5611_PROM_ADDRESS_READ_2      0xA4
#define MS5611_PROM_ADDRESS_READ_3      0xA6
#define MS5611_PROM_ADDRESS_READ_4      0xA8
#define MS5611_PROM_ADDRESS_READ_5      0xAA
#define MS5611_PROM_ADDRESS_READ_6      0xAC
#define MS5611_PROM_ADDRESS_READ_7      0xAE

#define MS5611_ADDR_CMD_D1_OSR256       0x40
#define MS5611_ADDR_CMD_D1_OSR512       0x42
#define MS5611_ADDR_CMD_D1_OSR1024      0x44
#define MS5611_ADDR_CMD_D1_OSR2048      0x46
#define MS5611_ADDR_CMD_D1_OSR4096      0x48

#define MS5611_ADDR_CMD_D2_OSR256       0x50
#define MS5611_ADDR_CMD_D2_OSR512       0x52
#define MS5611_ADDR_CMD_D2_OSR1024      0x54
#define MS5611_ADDR_CMD_D2_OSR2048      0x56
#define MS5611_ADDR_CMD_D2_OSR4096      0x58

#define MS5611_CONVERSION_TIME_OSR_256          1
#define MS5611_CONVERSION_TIME_OSR_512          2
#define MS5611_CONVERSION_TIME_OSR_1024         3
#define MS5611_CONVERSION_TIME_OSR_2048         5
#define MS5611_CONVERSION_TIME_OSR_4096         9

// Coefficients indexes for temperature and pressure computation
#define MS5611_PRESS_SENS_INDEX                 1
#define MS5611_PRESS_OFFSET_INDEX               2
#define MS5611_TEMP_COEFF_OF_PRESS_SEN_INDEX    3
#define MS5611_TEMP_COEFF_OF_PRESS_OFF_INDEX    4
#define MS5611_REF_TEMP_INDEX                   5
#define MS5611_TEMP_COEFF_OF_TEMP_INDEX         6
#define MS5611_CRC_INDEX                        7
#define MS5611_COEFF_NUMBERS                    8

// Hypsometric variables
#define CONVERT_C_TO_K                          273.15
#define INV_GAMMA                               1.0/5.257
#define TEMP_LAPSE_RATE                         0.0065

// Formulas
// #define ACCEL_CONVERT_UNITS(accel_val)   ((float)(accel_val*GRAVITY_EARTH)/SENSITIVITY_2G)
// #define GYRO_CONVERT_UNITS(gyro_val)     ((float)gyro_val/SENSITIVITY_2000)
// #define SQUARE(x)                        ((x)*(x))

// Static functions
static uint16_t eeprom_coeff[MS5611_COEFF_NUMBERS];
static uint32_t conversion_time[5] = {	MS5611_CONVERSION_TIME_OSR_256,
										MS5611_CONVERSION_TIME_OSR_512,
										MS5611_CONVERSION_TIME_OSR_1024,
										MS5611_CONVERSION_TIME_OSR_2048,
										MS5611_CONVERSION_TIME_OSR_4096};

/* Default to OSR of 1024 to reduce the self-heating effect of the sensor.*/
enum ms5611_resolution_osr {
	ms5611_resolution_osr_256 = 0,
	ms5611_resolution_osr_512,
	ms5611_resolution_osr_1024,
	ms5611_resolution_osr_2048,
	ms5611_resolution_osr_4096
};

/* Error status */
enum ms5611_status {
	ms5611_status_ok = 0,
	ms5611_status_no_i2c_acknowledge,
	ms5611_status_i2c_transfer_error,
	ms5611_status_crc_error
};


// /* Default value to ensure coefficients are read before converting temperature */
// bool ms5611_coeff_read = false;

/*! \brief 
 * \ingroup MS5611
 */
typedef struct ms5611_data {
    i2c_inst_t *i2c_address;
    spi_inst_t *spi_address;
    float sea_level_pressure;               // Sea level pressure
    float pressure_float;                   // Calculated temperature
    float baro_temp_float;                  // Calculated pressure
    float alt_float;                        // Calculated altitude
    float alt_avg;                          // Averaged altitude readings
    enum {READ_PRESSURE, READ_TEMPERATURE} ADC_state;           // Read state
    enum {MS5611_PENDING, MS5611_READY} data_ready;               // Serial comms
    enum {COEFF_ERROR, COEFF_VALID} ms5611_coeff_check;               // Serial comms
} ms5611_data_t;


/* Functions */
/*! \brief Breif explanation
 *  \ingroup MS5611 Sensor
 *
 * Detailed explanation
 *
 * \param value1 value comment
 * \param value2 value comment
 * \return return comments
 */
bool ms5611_i2c_init(ms5611_data_t *ms5611, i2c_inst_t *i2c, float *sea_level_pressure);


/**
 * \brief CRC check
 *
 * \param[in] uint16_t *: List of EEPROM coefficients
 * \param[in] uint8_t : crc to compare with
 *
 * \return bool : TRUE if CRC is OK, FALSE if KO
 */
bool ms5611_crc_check(uint16_t *n_prom, uint8_t crc);


/*! \brief Breif explanation
 *  \ingroup MS5611 Sensor
 *
 * Detailed explanation
 *
 * \param value1 value comment
 * \param value2 value comment
 * \return return comments
 */
bool MS5611_Avg(void);


/*! \brief Breif explanation
 *  \ingroup MS5611 Sensor
 *
 * Detailed explanation
 *
 * \param i2c value comment
 * \param MS5611_data value comment
 * \return return comments
 */
bool MS5611_Read(void);

#ifdef __cplusplus
}
#endif
#endif