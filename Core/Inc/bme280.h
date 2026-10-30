/*
 * bme280.h
 *
 *  Created on: Oct 25, 2026
 *      Author: antonioperedo
 */

#ifndef BME280_H_
#define BME280_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

#define BME280_EXPECTED_ID 0x60

typedef struct
{
	int32_t temperature_hundredths;
	uint32_t humidity_hundredths;
	uint32_t pressure_pascals;
} bme_measurements;

bool bme280_init(I2C_HandleTypeDef *i2c_handle);
bool bme280_read_id(uint8_t *chip_id);
bool bme280_read(bme_measurements *reading);

#endif /* BME280_H_ */
