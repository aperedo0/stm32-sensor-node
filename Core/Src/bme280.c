/*
 * bme280.c
 *
 *  Created on: Oct 25, 2026
 *      Author: antonioperedo
 */

#include "bme280.h"

#define BME280_I2C_ADDRESS             0x77
#define BME280_ID_REGISTER             0xD0
#define BME280_CALIBRATION_T_REGISTER  0x88
#define BME280_CTRL_MEAS_REGISTER      0xF4
#define BME280_TEMP_DATA_REGISTER      0xFA
#define BME280_NORMAL_MODE_SETTINGS    0x27
#define BME280_I2C_TIMEOUT_MS          100

static I2C_HandleTypeDef *bme280_i2c;
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

static bool read_registers(uint8_t first_register, uint8_t *destination, uint16_t count)
{
  return HAL_I2C_Mem_Read(bme280_i2c, BME280_I2C_ADDRESS << 1, first_register,
                          I2C_MEMADD_SIZE_8BIT, destination, count,
                          BME280_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool write_register(uint8_t register_address, uint8_t value)
{
  return HAL_I2C_Mem_Write(bme280_i2c, BME280_I2C_ADDRESS << 1, register_address,
                           I2C_MEMADD_SIZE_8BIT, &value, 1,
                           BME280_I2C_TIMEOUT_MS) == HAL_OK;
}

bool bme280_init(I2C_HandleTypeDef *i2c_handle)
{
  bme280_i2c = i2c_handle;

  uint8_t calibration_bytes[6];
  if (!read_registers(BME280_CALIBRATION_T_REGISTER, calibration_bytes, 6)) {
    return false;
  }

  dig_T1 = (uint16_t)((calibration_bytes[1] << 8) | calibration_bytes[0]);
  dig_T2 = (int16_t)((calibration_bytes[3] << 8) | calibration_bytes[2]);
  dig_T3 = (int16_t)((calibration_bytes[5] << 8) | calibration_bytes[4]);

  if (!write_register(BME280_CTRL_MEAS_REGISTER, BME280_NORMAL_MODE_SETTINGS)) {
    return false;
  }

  HAL_Delay(10);
  return true;
}

bool bme280_read_id(uint8_t *chip_id)
{
  return read_registers(BME280_ID_REGISTER, chip_id, 1);
}

bool bme280_read_temperature(int32_t *temperature_hundredths)
{
  uint8_t raw_bytes[3];
  if (!read_registers(BME280_TEMP_DATA_REGISTER, raw_bytes, 3)) {
    return false;
  }

  int32_t adc_T = ((int32_t)raw_bytes[0] << 12) | ((int32_t)raw_bytes[1] << 4) | (raw_bytes[2] >> 4);

  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  int32_t t_fine = var1 + var2;

  *temperature_hundredths = (t_fine * 5 + 128) >> 8;
  return true;
}
