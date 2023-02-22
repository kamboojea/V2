/**

 * @file sht4x_driver.h
 * @brief Header file for SHT4x sensor driver
 * @date Feb 17, 2023
 * @author ARahmatinia
*/
#ifndef INC_SHT4X_DRIVER_H_
#define INC_SHT4X_DRIVER_H_

#include "stm32f1xx_hal.h"

/**

 * @brief Structure that holds the SHT4x device information.
*/
typedef struct
{
 const char *name; // The name of the device
 I2C_HandleTypeDef *i2c_bus; // Pointer to the I2C bus handle (exp:hi2c2)
} sht4x_device_t;


/**
 * @brief Initializes the SHT4x device.
 * @param[in,out] me pointer to the SHT4x device structure
 * @param[in] name The name of the device
 * @param[in] i2c_bus pointer to the I2C bus handle (exp:hi2c2)
 * @return Returns the status of the initialization
*/
HAL_StatusTypeDef sht4x_init(sht4x_device_t * const me, const char *name, I2C_HandleTypeDef * const i2c_bus);


/**
 * @brief Starts temperature and humidity measurement for the SHT4x device.
 * @param[in] me pointer to the SHT4x device structure
 * @return Returns the status of starting the measurement
*/
HAL_StatusTypeDef sht4x_start_temperature_humidity_measurement(const sht4x_device_t * const me);


/**
 * @brief Reads the temperature measurement from the SHT4x device.
 * @param[in] me pointer to the SHT4x device structure
 * @param[out] temperature pointer to the location to store the temperature measurement
 * @return Returns the status of reading the temperature measurement
*/
HAL_StatusTypeDef sht4x_read_temperature_measurement(const sht4x_device_t * const me, float *temperature);


/**
 * @brief Reads the humidity measurement from the SHT4x device.
 * @param[in] me pointer to the SHT4x device structure
 * @param[out] humidity pointer to the location to store the humidity measurement
 * @return Returns the status of reading the humidity measurement
*/
HAL_StatusTypeDef sht4x_read_humidity_measurement(const sht4x_device_t * const me, float *humidity);


/**
 * @brief Soft reset the SHT4x device.
 * @param[in] me pointer to the SHT4x device structure
 * @return Returns the status of starting the measurement
*/
HAL_StatusTypeDef sht4x_soft_reset(const sht4x_device_t * const me);


#endif /* INC_SHT4X_DRIVER_H_ */
