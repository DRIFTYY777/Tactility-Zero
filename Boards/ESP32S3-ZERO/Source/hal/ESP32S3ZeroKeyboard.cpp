#include "ESP32S3ZeroKeyboard.h"
#include "ESP32S3ZeroConstants.h"
#include <Tactility/hal/i2c/I2c.h>
#include <Tactility/Tactility.h>

#define TAG "esp32s3_zero_keyboard"

#define ESP32S3_KEYBOARD_I2C_BUS_HANDLE I2C_NUM_0
#define ESP32S3_KEYBOARD_SLAVE_ADDRESS 0x20

// Replace hardcoded slave address with dynamic detection
static uint8_t detected_slave_address = ESP32S3_KEYBOARD_SLAVE_ADDRESS;

// Read 16-bit input state from PCA9555
static esp_err_t read_pca9555_inputs(uint16_t* input_state) {
    uint8_t port0 = 0, port1 = 0;

    // Read input port 0
    if (!tt::hal::i2c::masterReadRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, detected_slave_address, 0x00, &port0, 1, pdMS_TO_TICKS(100))) {
        return ESP_FAIL;
    }

    // Read input port 1
    if (!tt::hal::i2c::masterReadRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, detected_slave_address, 0x01, &port1, 1, pdMS_TO_TICKS(100))) {
        return ESP_FAIL;
    }

    // Combine both ports into 16-bit value
    *input_state = (port1 << 8) | port0;
    // Debug: log raw port values and combined state
    TT_LOG_I(TAG, "PCA9555 read: port0=0x%02X, port1=0x%02X, combined=0x%04X", port0, port1, *input_state);
    return ESP_OK;
}


// Configure PCA9555 pins as inputs
static esp_err_t configure_pca9555() {
    TT_LOG_I(TAG, "Configuring PCA9555 pins as inputs...");

    // Set configuration register 0 (pins 0-7) to all inputs (0xFF)
    uint8_t config_data = 0xFF;
    if (!tt::hal::i2c::masterWriteRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, detected_slave_address, 0x06, &config_data, 1, pdMS_TO_TICKS(1000))) {
        TT_LOG_E(TAG, "Failed to configure port 0");
        return ESP_FAIL;
    }

    // Set configuration register 1 (pins 8-15) to all inputs (0xFF)
    if (!tt::hal::i2c::masterWriteRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, detected_slave_address, 0x07, &config_data, 1, pdMS_TO_TICKS(1000))) {
        TT_LOG_E(TAG, "Failed to configure port 1");
        return ESP_FAIL;
    }

    TT_LOG_I(TAG, "PCA9555 configuration complete");
    return ESP_OK;
}


static inline bool keyboard_i2c_read(uint8_t* output) {
    uint16_t input_state;
    esp_err_t ret = read_pca9555_inputs(&input_state);
    if (ret == ESP_OK) {
        // Map button presses to key codes based on hardware connections
        // Buttons are active low, so pressed = 0, released = 1
        uint16_t pressed = ~input_state; // Invert to get pressed buttons as 1

        // Map specific pins to key codes
        if (pressed & (1 << 0)) *output = 1; // Up
        else if (pressed & (1 << 1))
            *output = 2; // Down
        else if (pressed & (1 << 2))
            *output = 3; // Right
        else if (pressed & (1 << 3))
            *output = 4; // Left
        else if (pressed & (1 << 5))
            *output = 5; // Menu
        else if (pressed & (1 << 6))
            *output = 6; // Back
        else if (pressed & (1 << 7))
            *output = 7; // OK
        else
            *output = 0; // No buttons pressed

        return true;
    }
    return false;
}


/**
 * The callback simulates press and release events for ESP32S3-ZERO keyboard.
 * LVGL currently works without those extra release events, but they
 * are implemented for correctness and future compatibility.
 *
 * @param indev
 * @param data
 */
static void keyboard_read_callback(TT_UNUSED lv_indev_t* indev, lv_indev_data_t* data) {
    static uint8_t last_buffer = 0x00;
    uint8_t read_buffer = 0x00;

    // Defaults
    data->key = 0;
    data->state = LV_INDEV_STATE_RELEASED;

    if (keyboard_i2c_read(&read_buffer)) {
        if (read_buffer == 0 && read_buffer != last_buffer) {
            TT_LOG_D(TAG, "Released %d", last_buffer);
            data->key = last_buffer;
            data->state = LV_INDEV_STATE_RELEASED;
        } else if (read_buffer != 0) {
            TT_LOG_D(TAG, "Pressed %d", read_buffer);
            data->key = read_buffer;
            data->state = LV_INDEV_STATE_PRESSED;
        }
    }

    last_buffer = read_buffer;
}

bool ESP32S3ZeroKeyboard::start(lv_display_t* display) {
    TT_LOG_I(TAG, "Starting ESP32S3-ZERO keyboard...");

    // Ensure I2C bus is started (ByTactility mode may have already started it)
    if (!tt::hal::i2c::isStarted(ESP32S3_KEYBOARD_I2C_BUS_HANDLE)) {
        if (!tt::hal::i2c::start(ESP32S3_KEYBOARD_I2C_BUS_HANDLE)) {
            TT_LOG_E(TAG, "Failed to start I2C bus %d", ESP32S3_KEYBOARD_I2C_BUS_HANDLE);
            return false;
        }
    }
    TT_LOG_I(TAG, "I2C bus %d is ready (SDA=GPIO%d, SCL=GPIO%d)", ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_ZERO_I2C_SDA, ESP32S3_ZERO_I2C_SCL);


    // Scan entire I2C bus and probe each device to detect PCA9555
    TT_LOG_I(TAG, "Scanning full I2C bus for PCA9555 devices...");
    bool found = false;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        if (tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, addr, pdMS_TO_TICKS(50))) {
            TT_LOG_D(TAG, "Probing device at address 0x%02X for PCA9555", addr);
            uint8_t test;
            // Attempt to read configuration register 0 (0x06)
            if (tt::hal::i2c::masterReadRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, addr, 0x06, &test, 1, pdMS_TO_TICKS(100))) {
                detected_slave_address = addr;
                found = true;
                TT_LOG_I(TAG, "PCA9555 detected at address 0x%02X", addr);
                break;
            }
        }
    }
    if (!found) {
        TT_LOG_E(TAG, "No PCA9555 detected on I2C bus");
        TT_LOG_I(TAG, "Devices responding on bus:");
        for (uint8_t addr = 1; addr < 127; ++addr) {
            if (tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, addr, pdMS_TO_TICKS(50))) {
                TT_LOG_I(TAG, "  - Device at 0x%02X", addr);
            }
        }
        return false;
    }
    if (detected_slave_address != ESP32S3_KEYBOARD_SLAVE_ADDRESS) {
        TT_LOG_W(TAG, "Using PCA9555 at non-default address 0x%02X", detected_slave_address);
    }
    TT_LOG_I(TAG, "PCA9555 ready at address 0x%02X", detected_slave_address);
    
    // Configure PCA9555 for input operation using detected address
    if (configure_pca9555() != ESP_OK) {
        TT_LOG_E(TAG, "Failed to configure PCA9555");
        return false;
    }

    deviceHandle = lv_indev_create();
    lv_indev_set_type(deviceHandle, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(deviceHandle, &keyboard_read_callback);
    lv_indev_set_display(deviceHandle, display);
    lv_indev_set_user_data(deviceHandle, this);

    // After ensuring bus is ready, scan for all devices
    TT_LOG_I(TAG, "Scanning I2C bus for all devices...");
    for (uint8_t addr = 1; addr < 127; ++addr) {
        if (tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, addr, pdMS_TO_TICKS(50))) {
            TT_LOG_I(TAG, "  - Device at 0x%02X", addr);
        }
    }

    TT_LOG_I(TAG, "ESP32S3-ZERO keyboard started successfully");
    return true;
}

bool ESP32S3ZeroKeyboard::stop() {
    if (deviceHandle) {
        lv_indev_delete(deviceHandle);
        deviceHandle = nullptr;
    }
    TT_LOG_I(TAG, "ESP32S3-ZERO keyboard stopped");
    return true;
}

bool ESP32S3ZeroKeyboard::isAttached() const {
    TT_LOG_I(TAG, "Checking for keyboard at I2C address 0x%02X on bus %d", ESP32S3_KEYBOARD_SLAVE_ADDRESS, ESP32S3_KEYBOARD_I2C_BUS_HANDLE);
    // Ensure bus is up before probing
    if (!tt::hal::i2c::isStarted(ESP32S3_KEYBOARD_I2C_BUS_HANDLE)) {
        if (!tt::hal::i2c::start(ESP32S3_KEYBOARD_I2C_BUS_HANDLE)) {
            TT_LOG_E(TAG, "Failed to start I2C bus %d in isAttached", ESP32S3_KEYBOARD_I2C_BUS_HANDLE);
            return false;
        }
    }
    // Scan entire I2C bus to detect devices
    TT_LOG_I(TAG, "isAttached: scanning I2C bus for devices...");
    bool foundKeyboard = false;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        if (tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, addr, pdMS_TO_TICKS(50))) {
            TT_LOG_I(TAG, "isAttached: device at 0x%02X", addr);
            if (addr == ESP32S3_KEYBOARD_SLAVE_ADDRESS) {
                foundKeyboard = true;
            }
        }
    }
    if (!foundKeyboard) {
        TT_LOG_E(TAG, "isAttached: PCA9555 not found at expected address 0x%02X", ESP32S3_KEYBOARD_SLAVE_ADDRESS);
    }
    return foundKeyboard;
}

std::shared_ptr<tt::hal::keyboard::KeyboardDevice> createKeyboard() {
    return std::make_shared<ESP32S3ZeroKeyboard>();
}
