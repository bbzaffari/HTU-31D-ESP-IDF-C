# HTU31D-ESP-IDF

**This code is shared publicly with no copyrights; anyone is free to use, modify, or redistribute it.**

This driver was developed entirely by me, Bruno Bavaresco Zaffari, as part of my undergraduate final thesis project in Computer Engineering. 

## Introduction

The HTU31D is a capacitive digital humidity and temperature sensor manufactured by TE Connectivity.  
It combines a polymer-based capacitive sensing element for relative humidity with a bandgap temperature sensor, providing long-term stability, high precision, and low power consumption — ideal for environmental monitoring, HVAC systems, weather stations, and IoT devices.

## Operational Ranges and Specifications

- **Temperature range:** -40°C to +125°C  
- **Humidity range:** 0% to 100% RH  
- **Typical accuracy:** ±2% RH, ±0.2°C  
- **Supply voltage:** 1.8 V to 3.6 V  
- **Interface:** I²C digital interface  
- **Features:** Integrated heater, configurable resolution, user-accessible registers

## Documentation

For detailed electrical characteristics, communication protocols, and performance specifications, refer to the official  
[HTU31D Datasheet](https://www.te.com/en/product-CAT-HSC0007.html) 

## Commands and Usage

This driver uses a subset of the available HTU31D sensor commands, focusing on essential functions for temperature and humidity measurements.

#### Commands actively used in this implementation

- `HTU31D_CONVERSION (0x40)` — Starts a temperature and humidity measurement.
- `HTU31D_READTEMPHUM (0x00)` — Reads the combined temperature and humidity data.
- `HTU31D_RESET (0x1E)` — Soft resets the sensor.

These are the core commands required for basic sensor operation and environmental monitoring.

#### Commands defined but not yet used

- `HTU31D_HEATERON (0x04)` / `HTU31D_HEATEROFF (0x02)` — Enable/disable the internal heater to reduce condensation.
- `CMD_MEAS_RH_HOLD (0xE5)` / `CMD_MEAS_T_HOLD (0xE3)` — Perform humidity or temperature measurements with I2C hold.
- `CMD_MEAS_RH_NOHOLD (0xF5)` / `CMD_MEAS_T_NOHOLD (0xF3)` — Perform measurements without holding the I2C bus.
- `CMD_READ_T (0xE0)` — Read the last temperature value without a new measurement.
- `CMD_RESET (0xFE)` — Alternate reset command (legacy).
- `CMD_READ_USER_REG (0xE7)` / `CMD_WRITE_USER_REG (0xE6)` — Access and configure the user register (resolution, heater enable).
- `CMD_WRITE_HEATER_REG (0x51)` / `CMD_READ_HEATER_REG (0x11)` — Advanced heater configuration.
- `CMD_READ_ID_1 (0x0FFA)` / `CMD_READ_ID_2 (0xC9FC)` — Retrieve unique device identifiers.
- `CMD_READ_FW_REV_1 (0xB884)` — Read firmware revision.
- `BIT_USER_REG_RES0`, `BIT_USER_REG_HTRE`, `BIT_USER_REG_RES1` — Bitmasks for user register manipulation.
- `HEATER_MASK`, `BV(x)` — Utilities for bit-level operations.

These commands are defined for potential future expansion, such as advanced device configuration, heater control, or device diagnostics. They are included to make the driver easier to extend and to document the full range of HTU31D capabilities.

## HTU31D Driver Functions

**htu31\_init**

```c
esp_err_t htu31_init(uint8_t i2c_port, gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz, i2c_master_bus_handle_t *bus_handle_out, i2c_master_dev_handle_t *dev_handle_out);
```

Initializes the I2C bus and adds the HTU31D device at address `0x40`. Probes the sensor using `HTU31_ADDR` and calls `htu31_reset()` internally.
**Commands used:** `HTU31D_RESET (0x1E)`

---

**htu31\_reset**

```c
esp_err_t htu31_reset(i2c_master_dev_handle_t dev);
```

Sends a soft reset command (`0x1E`) to the sensor to clear its internal state and reinitialize.
**Commands used:** `HTU31D_RESET (0x1E)`

---

**htu31\_measure\_all\_combined**

```c
esp_err_t htu31_measure_all_combined(i2c_master_dev_handle_t dev, float *temperature, float *humidity);
```

Performs a combined measurement:

1. Sends `HTU31D_CONVERSION (0x40)` to start,
2. Sends `HTU31D_READTEMPHUM (0x00)` to read back 6 bytes (temp + humidity + CRC).
   Processes raw values into °C and %RH, with CRC validation.
   **Commands used:** `HTU31D_CONVERSION (0x40)`, `HTU31D_READTEMPHUM (0x00)`

---

**htu31\_measure\_temperature**

```c
esp_err_t htu31_measure_temperature(i2c_master_dev_handle_t dev, float *temperature);
```

Wrapper around `htu31_measure_all_combined()`. Runs full combined measurement, but only returns the temperature value.
**Commands used:** same as `htu31_measure_all_combined`

---

**htu31\_measure\_humidity**

```c
esp_err_t htu31_measure_humidity(i2c_master_dev_handle_t dev, float *humidity);
```

Wrapper around `htu31_measure_all_combined()`. Runs full combined measurement, but only returns the humidity value.
**Commands used:** same as `htu31_measure_all_combined`

---

**\_htu31d\_crc\_check** (internal)

```c
bool _htu31d_crc_check(const uint8_t *data, size_t len, uint8_t crc);
```

Verifies the CRC of received sensor data using polynomial `0x31` (note: `CRC_POLY_DIVISOR` define is unused).
**Commands used:** none (pure data check)


---
---
## Minimal Example: HTU31D Single Reading

In addition to the main libraries (`freertos`, `esp_log`, `driver/i2c_master`) and the `htu31.h` header, this is the minimal code required to perform a single reading from the HTU31D sensor and print the result to the log.

Place the following inside `app_main`:

```c
htu31_init(0, 21, 22, 100000, &bus, &dev);

float t, rh;
if (htu31_measure_all_combined(dev, &t, &rh) == ESP_OK)
    ESP_LOGI("HTU31", "T:%.1fC H:%.1f%%", t, rh);
```

This initializes the sensor, performs one temperature and humidity measurement, and logs the values to the serial console.

No loop or additional configuration is required for this simple test.


---

## General Notes

* Heater functions (`HTU31D_HEATERON`, `HTU31D_HEATEROFF`) are defined but not yet implemented in functions.
* Other commands (`CMD_MEAS_*`, `CMD_RESET`, `CMD_READ_ID_*`) are defined but not currently used.
