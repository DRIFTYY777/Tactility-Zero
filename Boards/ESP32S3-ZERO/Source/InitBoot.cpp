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
    // esp_err_t isr_result = gpio_install_isr_service(0); // Use 0 for default priority
    // if (isr_result != ESP_OK && isr_result != ESP_ERR_INVALID_STATE) {
    //     TT_LOG_E(TAG, "Failed to install GPIO ISR service: %d", isr_result);
    //     // Continue anyway as it might already be installed
    // }

    // Initialize PWM backlight for the display
    if (!driver::pwmbacklight::init(ESP32S3_ZERO_LCD_PIN_BACKLIGHT)) {
        TT_LOG_E(TAG, "Failed to initialize backlight");
        return false;
    }


    

    TT_LOG_I(TAG, "ESP32S3-ZERO boot initialization complete");
    return true;
}
