/*
 * LSR_H3LIS200DLTR.c
 *
 *  Created on: 01 December 2022
 *      Author: Michael Venz
 *      References:
 *      https://www.st.com/content/ccc/resource/technical/document/datasheet/6c/e1/2a/9d/29/58/4e/25/DM00162194.pdf/files/DM00162194.pdf/jcr:content/translations/en.DM00162194.pdf
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
#include "LSR_H3LIS200DLTR.h"
#include "LSR_serial.h"

