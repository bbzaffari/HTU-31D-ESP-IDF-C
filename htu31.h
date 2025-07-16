#pragma once
#include <stdbool.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_timer.h>
#include <esp_err.h>
#include "esp_rom_sys.h"  // para esp_rom_delay_us


#define HTU31_ADDR            0x40
#define I2C_TIMEOUT_MS        1000
#define I2C_FREQ_HZ           400000
#define DELAY_MS              20

#define CMD_MEAS_RH_HOLD      0xE5
#define CMD_MEAS_T_HOLD       0xE3
#define CMD_MEAS_RH_NOHOLD    0xF5
#define CMD_MEAS_T_NOHOLD     0xF3
#define CMD_READ_T            0xE0
#define CMD_RESET             0xFE
#define CMD_READ_USER_REG     0xE7
#define CMD_WRITE_USER_REG    0xE6
#define CMD_WRITE_HEATER_REG  0x51
#define CMD_READ_HEATER_REG   0x11
#define CMD_READ_ID_1         0x0FFA
#define CMD_READ_ID_2         0xC9FC
#define CMD_READ_FW_REV_1     0xB884

#define BIT_USER_REG_RES0     0
#define BIT_USER_REG_HTRE     2
#define BIT_USER_REG_RES1     7

#define HEATER_MASK           0x0F
#define BV(x) (1 << (x))

/** Resoluções possíveis */
typedef enum {
    HTU31_RES_RH12_TEMP14 = 0,
    HTU31_RES_RH8_TEMP12  = 1,
    HTU31_RES_RH10_TEMP13 = 2,
    HTU31_RES_RH11_TEMP11 = 3
} htu31_resolution_t;

/** Calcula e verifica o CRC (retorna ESP_OK ou ESP_ERR_INVALID_CRC) */
esp_err_t htu31_crc_check(const uint8_t *data, size_t len, uint8_t crc);

/** Faz leitura completa de temperatura e umidade via comando combinado */
esp_err_t htu31_measure_all_combined(i2c_master_dev_handle_t dev, float *t, float *rh);

/** Mede apenas temperatura (modo NO HOLD) */
esp_err_t htu31_measure_temperature(i2c_master_dev_handle_t dev, float *t);

/** Mede apenas umidade (modo NO HOLD) */
esp_err_t htu31_measure_humidity(i2c_master_dev_handle_t dev, float *rh);

/** Executa um reset de software */
esp_err_t htu31_reset(i2c_master_dev_handle_t dev);

esp_err_t htu31_init(uint8_t i2c_port,
                     gpio_num_t sda,
                     gpio_num_t scl,
                     uint32_t freq_hz,
                     i2c_master_bus_handle_t *bus_handle_out,
                     i2c_master_dev_handle_t *dev_handle_out);