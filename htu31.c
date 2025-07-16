

#include "htu31.h"


static const char *TAG = "htu31d";

#define HTU31D_CONVERSION     0x40
#define HTU31D_READTEMPHUM    0x00
#define HTU31D_RESET          0x1E
#define HTU31D_HEATERON       0x04
#define HTU31D_HEATEROFF      0x02

#define DELAY_MS              20
#define I2C_TIMEOUT_MS        1000
#define CRC_POLY_DIVISOR      0x988000

//---------------------------------------------------------------
// Função interna para verificar o CRC dos dados
static uint8_t htu31d_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

bool _htu31d_crc_check(const uint8_t *data, size_t len, uint8_t crc) {
    return htu31d_crc8(data, len) == crc;
}

//---------------------------------------------------------------
// Reset do sensor (comando 0x1E)
esp_err_t htu31_reset(i2c_master_dev_handle_t dev) {
    uint8_t cmd = HTU31D_RESET;
    esp_err_t err = i2c_master_transmit(dev, &cmd, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Reset enviou NACK (tolerável): %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    return ESP_OK;
}

//---------------------------------------------------------------
// Leitura de temperatura e umidade combinadas (6 bytes + CRC)
esp_err_t htu31_measure_all_combined(i2c_master_dev_handle_t dev, float *temperature, float *humidity) {
    uint8_t conv_cmd = HTU31D_CONVERSION;
    esp_err_t err = i2c_master_transmit(dev, &conv_cmd, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10)); // tempo de conversão seguro

    uint8_t cmd = HTU31D_READTEMPHUM;
    uint8_t buf[6];

    err = i2c_master_transmit(dev, &cmd, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(1)); // pequena espera de resposta
    // Impressão dos dados crus
    // ESP_LOGI(TAG, "Bytes crus recebidos:");
    // ESP_LOGI(TAG, "Temp: %02X %02X [CRC: %02X]", buf[0], buf[1], buf[2]);
    // ESP_LOGI(TAG, "Hum : %02X %02X [CRC: %02X]", buf[3], buf[4], buf[5]);
    err = i2c_master_receive(dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    // ESP_LOGD(TAG, "Raw: %02X %02X %02X | %02X %02X %02X", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    if (!_htu31d_crc_check(&buf[0], 2, buf[2])) return ESP_ERR_INVALID_CRC;
    if (!_htu31d_crc_check(&buf[3], 2, buf[5])) return ESP_ERR_INVALID_CRC;

    uint16_t raw_temp = (buf[0] << 8) | buf[1];
    uint16_t raw_hum  = (buf[3] << 8) | buf[4];

    *temperature = ((float)raw_temp / 65535.0f) * 165.0f - 40.0f;
    *humidity    = ((float)raw_hum / 65535.0f) * 100.0f;

    return ESP_OK;
}

//---------------------------------------------------------------
// Wrapper: apenas temperatura
esp_err_t htu31_measure_temperature(i2c_master_dev_handle_t dev, float *temperature) {
    float dummy_humidity;
    return htu31_measure_all_combined(dev, temperature, &dummy_humidity);
}

//---------------------------------------------------------------
// Wrapper: apenas umidade
esp_err_t htu31_measure_humidity(i2c_master_dev_handle_t dev, float *humidity) {
    float dummy_temp;
    return htu31_measure_all_combined(dev, &dummy_temp, humidity);
}


esp_err_t htu31_init(uint8_t i2c_port,
                     gpio_num_t sda,
                     gpio_num_t scl,
                     uint32_t freq_hz,
                     i2c_master_bus_handle_t *bus_handle_out,
                     i2c_master_dev_handle_t *dev_handle_out)
{
    if (!bus_handle_out || !dev_handle_out) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    // 1. Configura o barramento I2C
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    err = i2c_new_master_bus(&bus_cfg, bus_handle_out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao criar barramento I2C: %s", esp_err_to_name(err));
        return err;
    }

    // 2. Adiciona o dispositivo HTU31
    i2c_device_config_t dev_cfg = {
        .device_address = 0x40,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = freq_hz,
    };

    err = i2c_master_bus_add_device(*bus_handle_out, &dev_cfg, dev_handle_out);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao adicionar HTU31 no barramento: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Verifica se o sensor responde
    err = i2c_master_probe(*bus_handle_out, HTU31_ADDR, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTU31 não detectado no endereço 0x%02X", HTU31_ADDR);
        return err;
    }

    ESP_LOGI(TAG, "HTU31 detectado no barramento com sucesso");

    // 4. Executa reset inicial
    err = htu31_reset(*dev_handle_out);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Reset falhou (pode ser tolerável): %s", esp_err_to_name(err));
    }

    return ESP_OK;
}
