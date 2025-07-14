#include "ESP32S3ZeroKeyboard.h"
#include "ESP32S3ZeroConstants.h"
#include <Tactility/hal/i2c/I2c.h>

#define TAG "esp32s3_zero_keyboard"

#define ESP32S3_KEYBOARD_I2C_BUS_HANDLE I2C_NUM_0
#define ESP32S3_KEYBOARD_SLAVE_ADDRESS 0x20

// Read 16-bit input state from PCA9555
static esp_err_t read_pca9555_inputs(uint16_t* input_state) {
    uint8_t port0 = 0, port1 = 0;

    // Read input port 0
    if (!tt::hal::i2c::masterReadRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_KEYBOARD_SLAVE_ADDRESS, 0x00, &port0, 1, pdMS_TO_TICKS(100))) {
        return ESP_FAIL;
    }

    // Read input port 1
    if (!tt::hal::i2c::masterReadRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_KEYBOARD_SLAVE_ADDRESS, 0x01, &port1, 1, pdMS_TO_TICKS(100))) {
        return ESP_FAIL;
    }

    // Combine both ports into 16-bit value
    *input_state = (port1 << 8) | port0;
    return ESP_OK;
}

// Test PCA9555 communication by reading a known register
static bool test_pca9555_communication() {
    TT_LOG_I(TAG, "Testing PCA9555 communication...");

    uint16_t input_state;
    esp_err_t ret = read_pca9555_inputs(&input_state);

    if (ret == ESP_OK) {
        TT_LOG_I(TAG, "PCA9555 communication successful! Current input state: 0x%04X", input_state);
        return true;
    } else {
        TT_LOG_E(TAG, "PCA9555 communication failed");
        return false;
    }
}


// Configure PCA9555 pins as inputs
static esp_err_t configure_pca9555() {
    TT_LOG_I(TAG, "Configuring PCA9555 pins as inputs...");

    // Set configuration register 0 (pins 0-7) to all inputs (0xFF)
    uint8_t config_data = 0xFF;
    if (!tt::hal::i2c::masterWriteRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_KEYBOARD_SLAVE_ADDRESS, 0x06, &config_data, 1, pdMS_TO_TICKS(1000))) {
        TT_LOG_E(TAG, "Failed to configure port 0");
        return ESP_FAIL;
    }

    // Set configuration register 1 (pins 8-15) to all inputs (0xFF)
    if (!tt::hal::i2c::masterWriteRegister(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_KEYBOARD_SLAVE_ADDRESS, 0x07, &config_data, 1, pdMS_TO_TICKS(1000))) {
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
    TT_LOG_I(TAG, "I2C Configuration: Bus=%d, SDA=GPIO%d, SCL=GPIO%d", ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_ZERO_I2C_SDA, ESP32S3_ZERO_I2C_SCL);

    // First, let's scan the entire I2C bus to see what devices are available
    TT_LOG_I(TAG, "Scanning I2C bus for all devices...");
    bool found_any = false;
    uint8_t common_pca9555_addresses[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27}; // Common PCA9555 addresses

    for (uint8_t addr = 1; addr < 127; addr++) {
        if (tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, addr, pdMS_TO_TICKS(100))) {
            TT_LOG_I(TAG, "Found I2C device at address 0x%02X", addr);
            found_any = true;

            // Check if this is a common PCA9555 address
            for (int i = 0; i < sizeof(common_pca9555_addresses); i++) {
                if (addr == common_pca9555_addresses[i]) {
                    TT_LOG_I(TAG, "  -> This is a common PCA9555 address!");
                    break;
                }
            }
        }
    }

    if (!found_any) {
        TT_LOG_E(TAG, "No I2C devices found on the bus! Check I2C wiring and pullups.");
        TT_LOG_E(TAG, "Expected SDA=GPIO%d, SCL=GPIO%d", ESP32S3_ZERO_I2C_SDA, ESP32S3_ZERO_I2C_SCL);
        return false;
    }

    // Check specifically for our keyboard device
    if (!tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_KEYBOARD_SLAVE_ADDRESS, pdMS_TO_TICKS(200))) {
        TT_LOG_E(TAG, "PCA9555 keyboard not found at I2C address 0x%02X", ESP32S3_KEYBOARD_SLAVE_ADDRESS);
        TT_LOG_E(TAG, "Available devices were listed above. Check if PCA9555 is properly connected and powered.");
        return false;
    }

    TT_LOG_I(TAG, "PCA9555 keyboard found at address 0x%02X", ESP32S3_KEYBOARD_SLAVE_ADDRESS);

    // Test basic communication with PCA9555
    if (!test_pca9555_communication()) {
        TT_LOG_E(TAG, "PCA9555 communication test failed");
        return false;
    }

    // Configure PCA9555 for input operation
    if (configure_pca9555() != ESP_OK) {
        TT_LOG_E(TAG, "Failed to configure PCA9555");
        return false;
    }

    deviceHandle = lv_indev_create();
    lv_indev_set_type(deviceHandle, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(deviceHandle, &keyboard_read_callback);
    lv_indev_set_display(deviceHandle, display);
    lv_indev_set_user_data(deviceHandle, this);

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
    // Use Tactility's thread-safe device detection with proper timeout
    return tt::hal::i2c::masterHasDeviceAtAddress(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, ESP32S3_KEYBOARD_SLAVE_ADDRESS, pdMS_TO_TICKS(200));
}

std::shared_ptr<tt::hal::keyboard::KeyboardDevice> createKeyboard() {
    return std::make_shared<ESP32S3ZeroKeyboard>();
}
