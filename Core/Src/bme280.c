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
#define BME280_DATA_REGISTER           0xF7
#define BME280_DATA_LENGTH             8
#define BME280_CALIBRATION_H_REGISTER  0xE1
#define BME280_CTRL_HUM_REGISTER       0xF2
#define BME280_HUMIDITY_SETTINGS       0x01

static I2C_HandleTypeDef *bme280_i2c;

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;

static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5;
static int16_t dig_P6, dig_P7, dig_P8, dig_P9;

static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5;
static int8_t dig_H6;

static int32_t t_fine;

static uint32_t compensate_humidity(int32_t adc_H)
{
    int32_t v = t_fine - ((int32_t)76800);

    v = (((((adc_H << 14) -
             (((int32_t)dig_H4) << 20) -
             (((int32_t)dig_H5) * v)) +
             ((int32_t)16384)) >> 15) *
         (((((((v * ((int32_t)dig_H6)) >> 10) *
              (((v * ((int32_t)dig_H3)) >> 11) +
               ((int32_t)32768))) >> 10) +
            ((int32_t)2097152)) *
            ((int32_t)dig_H2) + 8192) >> 14));

    v = (v - (((((v >> 15) * (v >> 15)) >> 7) *
               ((int32_t)dig_H1)) >> 4));

    v = (v < 0 ? 0 : v);

    v = (v > 419430400 ? 419430400 : v);

    uint32_t humidity_1024ths = (uint32_t)(v >> 12);

    return (humidity_1024ths * 100) >> 10;
}

static uint32_t compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;

    var2 = var1 * var1 * (int64_t)dig_P6;

    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);

    var2 = var2 + (((int64_t)dig_P4) << 35);

    var1 =
        ((var1 * var1 * (int64_t)dig_P3) >> 8) +
        ((var1 * (int64_t)dig_P2) << 12);

    var1 =
        (((((int64_t)1) << 47) + var1)) *
        ((int64_t)dig_P1) >> 33;

    if (var1 == 0)
    {
        return 0;
    }

    p = 1048576 - adc_P;

    p = (((p << 31) - var2) * 3125) / var1;

    var1 =
        (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;

    var2 =
        (((int64_t)dig_P8) * p) >> 19;

    p =
        ((p + var1 + var2) >> 8) +
        (((int64_t)dig_P7) << 4);

    return (uint32_t)(p >> 8);
}

static int32_t compensate_temperature(int32_t adc_T)
{
    int32_t var1 =
        ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
        ((int32_t)dig_T2)) >> 11;

    int32_t var2 =
        (((((adc_T >> 4) - ((int32_t)dig_T1)) *
        ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
        ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    return (t_fine * 5 + 128) >> 8;
}

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

static uint16_t little_endian_u16(const uint8_t *bytes)
{
	uint16_t result = (bytes[1] << 8) | bytes[0];

	return result;
}

static int16_t little_endian_s16(const uint8_t *bytes)
{
	int16_t result = little_endian_u16(bytes);
	return result;
}

bool bme280_init(I2C_HandleTypeDef *i2c_handle)
{
  bme280_i2c = i2c_handle;

  uint8_t calibration_a[26];
  uint8_t calibration_b[7];

  if (!read_registers(BME280_CALIBRATION_T_REGISTER, calibration_a, 26)) { return false; }
  if (!read_registers(BME280_CALIBRATION_H_REGISTER, calibration_b, 7)) { return false; }

  dig_T1 = (uint16_t)((calibration_a[1] << 8) | calibration_a[0]);
  dig_T2 = (int16_t)((calibration_a[3] << 8) | calibration_a[2]);
  dig_T3 = (int16_t)((calibration_a[5] << 8) | calibration_a[4]);

  dig_P1 = little_endian_u16(&calibration_a[6]);
  dig_P2 = little_endian_s16(&calibration_a[8]);
  dig_P3 = little_endian_s16(&calibration_a[10]);
  dig_P4 = little_endian_s16(&calibration_a[12]);
  dig_P5 = little_endian_s16(&calibration_a[14]);
  dig_P6 = little_endian_s16(&calibration_a[16]);
  dig_P7 = little_endian_s16(&calibration_a[18]);
  dig_P8 = little_endian_s16(&calibration_a[20]);
  dig_P9 = little_endian_s16(&calibration_a[22]);

  dig_H1 = calibration_a[25];
  dig_H2 = little_endian_s16(&calibration_b[0]);
  dig_H3 = calibration_b[2];
  dig_H4 = (int16_t)(((int8_t)calibration_b[3] * 16) | (calibration_b[4] & 0x0F));
  dig_H5 = (int16_t)(((int8_t)calibration_b[5] * 16) | (calibration_b[4] >> 4));
  dig_H6 = (int8_t)calibration_b[6];

  if (!write_register(BME280_CTRL_HUM_REGISTER, BME280_HUMIDITY_SETTINGS)) {
      return false;
    }
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

bool bme280_read(bme_measurements *reading)
{
	uint8_t raw[8];

	if(!read_registers(BME280_DATA_REGISTER, raw, BME280_DATA_LENGTH)){ return false; }

    int32_t adc_P =
        ((int32_t)raw[0] << 12) |
        ((int32_t)raw[1] << 4)  |
        (raw[2] >> 4);

    int32_t adc_T =
        ((int32_t)raw[3] << 12) |
        ((int32_t)raw[4] << 4)  |
        (raw[5] >> 4);

    int32_t adc_H =
        ((int32_t)raw[6] << 8)  |
        raw[7];

    reading->temperature_hundredths = compensate_temperature(adc_T);
    reading->pressure_pascals = compensate_pressure(adc_P);
    reading->humidity_hundredths = compensate_humidity(adc_H);

    return true;
}
