#include "ESP32S3ZeroKeyboard.h"
#include "ESP32S3ZeroConstants.h"
#include <Tactility/hal/i2c/I2c.h>
#include <driver/i2c.h>

#define TAG "esp32s3_zero_keyboard"

#define ESP32S3_KEYBOARD_I2C_BUS_HANDLE I2C_NUM_0
#define ESP32S3_KEYBOARD_SLAVE_ADDRESS 0x20
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

static_assert(ESP32S3_ZERO_I2C_SDA == GPIO_NUM_8, "I2C SDA should be GPIO8");
static_assert(ESP32S3_ZERO_I2C_SCL == GPIO_NUM_9, "I2C SCL should be GPIO9");

static bool i2c_initialized = false;

// Initialize I2C master for PCA9555 communication
static esp_err_t init_i2c_master() {
    if (i2c_initialized) {
        return ESP_OK;
    }


    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = ESP32S3_ZERO_I2C_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = ESP32S3_ZERO_I2C_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;


    esp_err_t ret = i2c_param_config(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, &conf);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_initialized = true;
    TT_LOG_I(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// Configure PCA9555 pins as inputs
static esp_err_t configure_pca9555() {
    TT_LOG_I(TAG, "Configuring PCA9555 pins as inputs...");

    // Set configuration register 0 (pins 0-7) to all inputs (0xFF)
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x06, true); // Configuration register 0
    i2c_master_write_byte(cmd, 0xFF, true); // All pins as inputs
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to configure port 0: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set configuration register 1 (pins 8-15) to all inputs (0xFF)
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x07, true); // Configuration register 1
    i2c_master_write_byte(cmd, 0xFF, true); // All pins as inputs
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to configure port 1: %s", esp_err_to_name(ret));
        return ret;
    }

    TT_LOG_I(TAG, "PCA9555 configuration complete");
    return ESP_OK;
}

// Read 16-bit input state from PCA9555
static esp_err_t read_pca9555_inputs(uint16_t* input_state) {
    uint8_t port0 = 0, port1 = 0;

    // Read input port 0
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true); // Input port 0
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &port0, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    // Read input port 1
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x01, true); // Input port 1
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &port1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    // Combine both ports into 16-bit value
    *input_state = (port1 << 8) | port0;
    return ESP_OK;
}

// Test PCA9555 communication
static bool test_pca9555_communication() {
    TT_LOG_I(TAG, "Testing PCA9555 communication...");

    uint16_t input_state;
    esp_err_t ret = read_pca9555_inputs(&input_state);

    if (ret == ESP_OK) {
        TT_LOG_I(TAG, "PCA9555 communication successful! Input state: 0x%04X", input_state);
        return true;
    } else {
        TT_LOG_E(TAG, "PCA9555 communication failed: %s", esp_err_to_name(ret));
        return false;
    }
}

static inline bool keyboard_i2c_read(uint8_t* output) {
    uint16_t input_state;
    esp_err_t ret = read_pca9555_inputs(&input_state);
    if (ret == ESP_OK) {
        // Map button presses to key codes based on hardware connections
        // Buttons are active low, so pressed = 0, released = 1
        uint16_t pressed = ~input_state; // Invert to get pressed buttons as 1

        // Map specific pins to key codes (adjust based on your hardware)
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

// Function to scan I2C bus for debugging
static void scanI2CBus() {
    TT_LOG_I(TAG, "========================================");
    TT_LOG_I(TAG, "Scanning I2C bus %d for devices...", ESP32S3_KEYBOARD_I2C_BUS_HANDLE);
    TT_LOG_I(TAG, "Expected I2C pins: SDA=GPIO%d, SCL=GPIO%d", ESP32S3_ZERO_I2C_SDA, ESP32S3_ZERO_I2C_SCL);
    TT_LOG_I(TAG, "Looking for keyboard at address 0x%02X", ESP32S3_KEYBOARD_SLAVE_ADDRESS);
    TT_LOG_I(TAG, "========================================");

    bool found = false;
    uint8_t devices_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t result = i2c_master_cmd_begin(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (result == ESP_OK) {
            TT_LOG_I(TAG, "Found device at address 0x%02X", addr);
            devices_found++;
            if (addr == ESP32S3_KEYBOARD_SLAVE_ADDRESS) {
                found = true;
            }
        }
    }

    if (devices_found == 0) {
        TT_LOG_W(TAG, "No I2C devices found! Check wiring.");
    } else {
        TT_LOG_I(TAG, "Found %d I2C device(s)", devices_found);
        if (found) {
            TT_LOG_I(TAG, "PCA9555 keyboard found at expected address 0x%02X", ESP32S3_KEYBOARD_SLAVE_ADDRESS);
        } else {
            TT_LOG_W(TAG, "PCA9555 keyboard NOT found at expected address 0x%02X", ESP32S3_KEYBOARD_SLAVE_ADDRESS);
        }
    }

    TT_LOG_I(TAG, "========================================");
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

    // Initialize I2C master
    esp_err_t ret = init_i2c_master();
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to initialize I2C master");
        return false;
    }

    // Scan I2C bus to see what's available
    scanI2CBus();

    // Test communication with PCA9555
    if (!test_pca9555_communication()) {
        TT_LOG_E(TAG, "PCA9555 communication test failed");
        return false;
    }

    // Configure PCA9555 pins as inputs
    ret = configure_pca9555();
    if (ret != ESP_OK) {
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

    // Simple I2C ping test using direct ESP-IDF commands
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP32S3_KEYBOARD_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t result = i2c_master_cmd_begin(ESP32S3_KEYBOARD_I2C_BUS_HANDLE, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (result == ESP_OK) {
        TT_LOG_I(TAG, "ESP32S3-ZERO keyboard detected successfully");
        return true;
    } else {
        TT_LOG_E(TAG, "ESP32S3-ZERO keyboard not detected: %s", esp_err_to_name(result));
        return false;
    }
}

std::shared_ptr<tt::hal::keyboard::KeyboardDevice> createKeyboard() {
    return std::make_shared<ESP32S3ZeroKeyboard>();
}
