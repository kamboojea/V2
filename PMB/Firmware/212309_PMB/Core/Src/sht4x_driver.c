/**
 * @file sht4x_driver.c
 * @brief Implementation file for SHT4x sensor driver
 * @date Feb 17, 2023
 * @author ARahmatinia
 */


#include "sht4x_driver.h"

/*==============================================================================
 * Defines
 */
#define SHT4x_ADDR 0x44 /**< SHT4x sensor address */
#define SHT4x_HIGH_PRECISION_COMMAND 0xFD /**< Command for high precision measurement */
#define SHT4x_SOFT_RESET_COMMAND 0x94 /**< Command for soft resetting the sensor */
#define SHT4x_SERIAL_NUMBER_COMMAND 0x89
#define SHT4x_SERIAL_NUMBER_CRC 0x31
#define TIMEOUT 100 /**< Timeout for I2C operations in milliseconds */
#define DATA_SIZE  6 /**< Size of data buffer for I2C communication */



HAL_StatusTypeDef sht4x_init(sht4x_device_t * const me, const char *name, I2C_HandleTypeDef * const i2c_bus)
{
    me->name = name;
    me->i2c_bus = i2c_bus;
    return HAL_OK;
}


HAL_StatusTypeDef sht4x_start_temperature_humidity_measurement(const sht4x_device_t * const me)
{
	uint8_t command = SHT4x_HIGH_PRECISION_COMMAND;
	return HAL_I2C_Master_Transmit(me -> i2c_bus, SHT4x_ADDR << 1, &command, 1,TIMEOUT);
}


HAL_StatusTypeDef sht4x_read_temperature_measurement(const sht4x_device_t * const me, float *temperature)
{
	uint8_t data[DATA_SIZE] = {0};

	HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(me -> i2c_bus,  SHT4x_ADDR << 1,data, DATA_SIZE, TIMEOUT);
	*temperature = (- 45.0 + (175.0 *(float)((data[0]<<8) | data[1])/ 65535.0)) ;
	return ret;
}


HAL_StatusTypeDef sht4x_read_humidity_measurement(const sht4x_device_t * const me, float *humidity)
{
	uint8_t data[DATA_SIZE] = {0};

	HAL_StatusTypeDef ret = HAL_I2C_Master_Receive(me -> i2c_bus,  SHT4x_ADDR << 1,data, DATA_SIZE, TIMEOUT);
	*humidity = ((100.0 * (float)((data[3] << 8) | data[4])) / 65535.0);
	return ret;
}


HAL_StatusTypeDef sht4x_soft_reset(const sht4x_device_t * const me)
{
	uint8_t command = SHT4x_SOFT_RESET_COMMAND;
	return HAL_I2C_Master_Transmit(me -> i2c_bus, SHT4x_ADDR << 1, &command, 1,TIMEOUT);
}

