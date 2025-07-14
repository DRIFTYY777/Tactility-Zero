/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "PCA9555.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define I2C_MASTER_SCL_IO 9 // GPIO number for I2C SCL
#define I2C_MASTER_SDA_IO 8 // GPIO number for I2C SDA
#define I2C_MASTER_NUM 0
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define PCA9555_ADDR 0x20 // I2C address of PCA9555 (Make sure it matches your hardware)
#define INTERRUPT_PIN GPIO_NUM_4 // GPIO to which the interrupt pin is connected

static const char* TAG = "PCA9555_EXAMPLE";

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void i2c_scanner(void) {
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t result = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
            devices_found++;
        }
    }

    if (devices_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found! Check wiring.");
    } else {
        ESP_LOGI(TAG, "Found %d I2C device(s)", devices_found);
    }
}

// Simple test function to read PCA9555 registers directly
static esp_err_t test_pca9555_direct(void) {
    ESP_LOGI(TAG, "Testing PCA9555 direct register access...");

    // Read input port 0 (register 0x00)
    uint8_t input_port0 = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Input port 0 register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &input_port0, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read input port 0: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read input port 1 (register 0x01)
    uint8_t input_port1 = 0;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x01, true); // Input port 1 register
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &input_port1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read input port 1: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Input Port 0: 0x%02X, Input Port 1: 0x%02X", input_port0, input_port1);
    ESP_LOGI(TAG, "Combined 16-bit input: 0x%04X", (input_port1 << 8) | input_port0);

    return ESP_OK;
}

void pca9555_task(void* pvParameters) {
    // Test direct communication first
    if (test_pca9555_direct() != ESP_OK) {
        ESP_LOGE(TAG, "Direct PCA9555 communication failed!");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Direct PCA9555 communication successful!");

    // Configure all pins as inputs by writing 0xFF to both configuration registers
    ESP_LOGI(TAG, "Configuring all pins as inputs...");

    // Set configuration register 0 (pins 0-7) to all inputs (0xFF)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x06, true); // Configuration register 0
    i2c_master_write_byte(cmd, 0xFF, true); // All pins as inputs
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure port 0: %s", esp_err_to_name(ret));
    }

    // Set configuration register 1 (pins 8-15) to all inputs (0xFF)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x07, true); // Configuration register 1
    i2c_master_write_byte(cmd, 0xFF, true); // All pins as inputs
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure port 1: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "Configuration complete. Starting button monitoring...");

    uint16_t last_state = 0xFFFF;

    while (1) {
        // Read both input ports
        uint8_t port0 = 0, port1 = 0;

        // Read input port 0
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x00, true); // Input port 0
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &port0, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error reading port 0");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Read input port 1
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, 0x01, true); // Input port 1
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (PCA9555_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &port1, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error reading port 1");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Combine both ports into 16-bit value
        uint16_t current_state = (port1 << 8) | port0;

        // Check if state changed
        if (current_state != last_state) {
            ESP_LOGI(TAG, "Input state: 0x%04X (Port0: 0x%02X, Port1: 0x%02X)", current_state, port0, port1);

            // Detect button presses (transitions from 1 to 0)
            uint16_t pressed = last_state & ~current_state;

            if (pressed & (1 << 0))
                ESP_LOGI(TAG, "Button: Up");
            if (pressed & (1 << 1))
                ESP_LOGI(TAG, "Button: Down");
            if (pressed & (1 << 2))
                ESP_LOGI(TAG, "Button: Right");
            if (pressed & (1 << 3))
                ESP_LOGI(TAG, "Button: Left");
            if (pressed & (1 << 4))
                ESP_LOGI(TAG, "Button: Unused");
            if (pressed & (1 << 5))
                ESP_LOGI(TAG, "Button: Menu");
            if (pressed & (1 << 6))
                ESP_LOGI(TAG, "Button: Back");
            if (pressed & (1 << 7))
                ESP_LOGI(TAG, "Button: OK");
            if (pressed & (1 << 8))
                ESP_LOGI(TAG, "Button: T_R");
            if (pressed & (1 << 9))
                ESP_LOGI(TAG, "Button: A");
            if (pressed & (1 << 10))
                ESP_LOGI(TAG, "Button: B");
            if (pressed & (1 << 11))
                ESP_LOGI(TAG, "Button: Y");
            if (pressed & (1 << 12))
                ESP_LOGI(TAG, "Button: X");
            if (pressed & (1 << 13))
                ESP_LOGI(TAG, "Button: T_L");

            last_state = current_state;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C...");
    ESP_ERROR_CHECK(i2c_master_init());

    // Scan I2C bus for devices
    i2c_scanner();

    ESP_LOGI(TAG, "Creating PCA9555 task...");
    xTaskCreate(pca9555_task, "pca9555_task", 4096, NULL, 10, NULL);
}
