#include "InitBoot.h"
#include "hal/ESP32S3ZeroConstants.h"

#include <PwmBacklight.h>
#include <Tactility/Log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "esp32s3_zero_init"

bool initBoot() {
    // Initialize GPIO ISR service for SD card
    esp_err_t isr_result = gpio_install_isr_service(0); // Use 0 for default priority
    if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
        TT_LOG_E(TAG, "Failed to install GPIO ISR service: %d", isr_result);
        // Continue anyway as it might already be installed
    }

    // Initialize PWM backlight for the display
    if (!driver::pwmbacklight::init(ESP32S3_ZERO_LCD_PIN_BACKLIGHT)) {
        TT_LOG_E(TAG, "Failed to initialize backlight");
        return false;
    }

    // Configure I2C pins with external pull-ups for keyboard GPIO expander
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; // Open drain mode
    io_conf.pin_bit_mask = (1ULL << ESP32S3_ZERO_I2C_SDA) | (1ULL << ESP32S3_ZERO_I2C_SCL);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Perform I2C bus recovery in case it's stuck
    TT_LOG_I(TAG, "Performing I2C bus recovery");

    // Set SCL as output high
    gpio_set_level(ESP32S3_ZERO_I2C_SCL, 1);

    // Toggle SCL 9 times to release any stuck device
    for (int i = 0; i < 9; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(ESP32S3_ZERO_I2C_SCL, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_set_level(ESP32S3_ZERO_I2C_SCL, 1);
    }

    // Generate STOP condition (SDA from low to high while SCL is high)
    gpio_set_level(ESP32S3_ZERO_I2C_SDA, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(ESP32S3_ZERO_I2C_SCL, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level(ESP32S3_ZERO_I2C_SDA, 1);

    TT_LOG_I(TAG, "I2C bus recovery completed");

    TT_LOG_I(TAG, "ESP32S3-ZERO boot initialization complete");
    return true;
}
