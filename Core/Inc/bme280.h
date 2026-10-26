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

bool bme280_init(I2C_HandleTypeDef *i2c_handle);
bool bme280_read_id(uint8_t *chip_id);
bool bme280_read_temperature(int32_t *temperature_hundretdths);

#endif /* BME280_H_ */
