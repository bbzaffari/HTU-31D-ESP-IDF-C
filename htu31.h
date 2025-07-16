#pragma once
#include <stdbool.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_timer.h>
#include <esp_err.h>
#include "esp_rom_sys.h" 

#define HTU31_ADDR            0x40

/** Possible resolutions */
typedef enum {
    HTU31_RES_RH12_TEMP14 = 0,   // Humidity 12-bit, Temperature 14-bit
    HTU31_RES_RH8_TEMP12  = 1,   // Humidity 8-bit, Temperature 12-bit
    HTU31_RES_RH10_TEMP13 = 2,   // Humidity 10-bit, Temperature 13-bit
    HTU31_RES_RH11_TEMP11 = 3    // Humidity 11-bit, Temperature 11-bit
} htu31_resolution_t;

/** Computes and checks CRC (returns ESP_OK or ESP_ERR_INVALID_CRC) */
esp_err_t htu31_crc_check(const uint8_t *data, size_t len, uint8_t crc);

/** Reads combined temperature and humidity using combined command */
esp_err_t htu31_measure_all_combined(i2c_master_dev_handle_t dev, float *t, float *rh);

/** Measures only temperature (NO HOLD mode) */
esp_err_t htu31_measure_temperature(i2c_master_dev_handle_t dev, float *t);

/** Measures only humidity (NO HOLD mode) */
esp_err_t htu31_measure_humidity(i2c_master_dev_handle_t dev, float *rh);

/** Executes a software reset */
esp_err_t htu31_reset(i2c_master_dev_handle_t dev);

/** Initializes I2C bus and HTU31 device */
esp_err_t htu31_init(uint8_t i2c_port,
                     gpio_num_t sda,
                     gpio_num_t scl,
                     uint32_t freq_hz,
                     i2c_master_bus_handle_t *bus_handle_out,
                     i2c_master_dev_handle_t *dev_handle_out);
