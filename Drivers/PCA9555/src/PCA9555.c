#include "PCA9555.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <rom/gpio.h>

#define I2C_MASTER_TIMEOUT_MS 1000

#define PCA9555_LIB_TAG "PCA9555"
#define PCA9555_IO_NUM 16 // PCA9555 has 16 IO pins (2 ports x 8 pins)

#ifdef __cplusplus
extern "C" {
#endif

/**
     * @brief PCA9555 Internal configuration and pin registers
     */
typedef enum {
    PCA9555_REG_INPUT_PORT_0, // Input port 0
    PCA9555_REG_INPUT_PORT_1, // Input port 1
    PCA9555_REG_OUTPUT_PORT_0, // Output port 0
    PCA9555_REG_OUTPUT_PORT_1, // Output port 1
    PCA9555_REG_POLARITY_INVERSION_0, // Polarity inversion port 0
    PCA9555_REG_POLARITY_INVERSION_1, // Polarity inversion port 1
    PCA9555_REG_CONFIGURATION_0, // Configuration port 0
    PCA9555_REG_CONFIGURATION_1 // Configuration port 1
} PCA9555_REGISTER;

/**
     * @brief Default PCA9555 interrupt task
     */
void PCA9555_default_interrupt_task(void* pvParameters) {
    ESP_LOGW(PCA9555_LIB_TAG, "No interrupt task defined! Using standard PCA9555 interrupt task!");
    PCA9555_IO_EXP* io_exp = (PCA9555_IO_EXP*)pvParameters;
    uint32_t io_num;
    while (1) {
        if (xTaskNotifyWait(0, 0, &io_num, portTICK_PERIOD_MS) == pdTRUE) {
            uint16_t input_status = get_pca9555_all_io_pin_input_status(io_exp);
            printf("Current input status (pin : status):\n");
            for (uint8_t i = 0; i < PCA9555_IO_NUM; i++)
                printf("P%d : %d\n", i, (input_status & (1 << i)) == (1 << i));
        }
    }
}

/**
     *  @brief PCA9555 interrupt handler
     */
static void IRAM_ATTR PCA9555_interrupt_handler(void* args) {
    PCA9555_IO_EXP* io_exp = (PCA9555_IO_EXP*)args;
    xTaskNotifyFromISR(*io_exp->interrupt_task, 0, eNoAction, 0);
}

/**
     * @brief Setup PCA9555 interrupts
     */
void setup_pca9555_interrupt_handler(PCA9555_IO_EXP* io_exp) {
    if (io_exp->interrupt_task == NULL) {
        xTaskCreate(
            PCA9555_default_interrupt_task, /* Function that implements the task. */
            "PCA9555_INT_TASK", /* Text name for the task. */
            2048, /* Stack size in words, not bytes. */
            (void*)io_exp, /* Parameter passed into the task. */
            10, /* Priority at which the task is created. */
            io_exp->interrupt_task
        ); /* Used to pass out the created task's handle. */
    }

    gpio_pad_select_gpio(io_exp->interrupt_pin);
    gpio_set_direction(io_exp->interrupt_pin, GPIO_MODE_INPUT);
    gpio_intr_enable(io_exp->interrupt_pin);

    gpio_set_intr_type(io_exp->interrupt_pin, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(io_exp->interrupt_pin, PCA9555_interrupt_handler, (void*)io_exp);
}

esp_err_t write_pca9555_reg(PCA9555_IO_EXP* io_exp, PCA9555_REGISTER cmd, uint8_t data) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (io_exp->I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_write_byte(cmd_handle, data, true);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(io_exp->i2c_master_port, cmd_handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

esp_err_t read_pca9555_reg(PCA9555_IO_EXP* io_exp, PCA9555_REGISTER cmd, uint8_t* read_buff) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (io_exp->I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd_handle, cmd, true);
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (io_exp->I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd_handle, read_buff, I2C_MASTER_NACK);
    i2c_master_stop(cmd_handle);
    esp_err_t ret = i2c_master_cmd_begin(io_exp->i2c_master_port, cmd_handle, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd_handle);
    return ret;
}

int16_t get_pca9555_all_io_pin_input_status(PCA9555_IO_EXP* io_exp) {
    uint8_t port0 = 0, port1 = 0;
    esp_err_t status;

    status = read_pca9555_reg(io_exp, PCA9555_REG_INPUT_PORT_0, &port0);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    status = read_pca9555_reg(io_exp, PCA9555_REG_INPUT_PORT_1, &port1);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    // Combine port0 and port1 into 16-bit result
    return (port1 << 8) | port0;
}

int16_t get_io_pin_input_status(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin) {
    uint8_t port;
    PCA9555_REGISTER reg;
    uint8_t pin_in_port;

    // Determine which port and pin within that port
    if (io_pin < 8) {
        reg = PCA9555_REG_INPUT_PORT_0;
        pin_in_port = io_pin;
    } else if (io_pin < 16) {
        reg = PCA9555_REG_INPUT_PORT_1;
        pin_in_port = io_pin - 8;
    } else {
        return PCA9555_ERROR; // Invalid pin
    }

    esp_err_t status = read_pca9555_reg(io_exp, reg, &port);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    return (port & (1 << pin_in_port)) ? 1 : 0;
}

int16_t get_all_io_pin_direction(PCA9555_IO_EXP* io_exp) {
    uint8_t port0 = 0, port1 = 0;
    esp_err_t status;

    status = read_pca9555_reg(io_exp, PCA9555_REG_CONFIGURATION_0, &port0);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    status = read_pca9555_reg(io_exp, PCA9555_REG_CONFIGURATION_1, &port1);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    // Combine port0 and port1 into 16-bit result
    return (port1 << 8) | port0;
}

int16_t get_io_pin_direction(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin) {
    uint8_t port;
    PCA9555_REGISTER reg;
    uint8_t pin_in_port;

    // Determine which port and pin within that port
    if (io_pin < 8) {
        reg = PCA9555_REG_CONFIGURATION_0;
        pin_in_port = io_pin;
    } else if (io_pin < 16) {
        reg = PCA9555_REG_CONFIGURATION_1;
        pin_in_port = io_pin - 8;
    } else {
        return PCA9555_ERROR; // Invalid pin
    }

    esp_err_t status = read_pca9555_reg(io_exp, reg, &port);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    return (port & (1 << pin_in_port)) ? 1 : 0;
}

int16_t get_all_io_polarity_inversion(PCA9555_IO_EXP* io_exp) {
    uint8_t port0 = 0, port1 = 0;
    esp_err_t status;

    status = read_pca9555_reg(io_exp, PCA9555_REG_POLARITY_INVERSION_0, &port0);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    status = read_pca9555_reg(io_exp, PCA9555_REG_POLARITY_INVERSION_1, &port1);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    // Combine port0 and port1 into 16-bit result
    return (port1 << 8) | port0;
}

int16_t get_io_pin_polarity_inversion(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin) {
    uint8_t port;
    PCA9555_REGISTER reg;
    uint8_t pin_in_port;

    // Determine which port and pin within that port
    if (io_pin < 8) {
        reg = PCA9555_REG_POLARITY_INVERSION_0;
        pin_in_port = io_pin;
    } else if (io_pin < 16) {
        reg = PCA9555_REG_POLARITY_INVERSION_1;
        pin_in_port = io_pin - 8;
    } else {
        return PCA9555_ERROR; // Invalid pin
    }

    esp_err_t status = read_pca9555_reg(io_exp, reg, &port);
    if (status != ESP_OK)
        return PCA9555_ERROR;

    return (port & (1 << pin_in_port)) ? 1 : 0;
}

esp_err_t set_all_pca9555_io_pins_direction(PCA9555_IO_EXP* io_exp, PCA9555_PORT_DIRECTION properties) {
    uint8_t dir = (properties == PCA9555_OUTPUT) ? 0x00 : 0xFF;
    esp_err_t status;

    status = write_pca9555_reg(io_exp, PCA9555_REG_CONFIGURATION_0, dir);
    if (status != ESP_OK)
        return status;

    status = write_pca9555_reg(io_exp, PCA9555_REG_CONFIGURATION_1, dir);
    return status;
}

esp_err_t set_pca9555_io_pin_direction(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin, PCA9555_PORT_DIRECTION properties) {
    uint8_t port;
    PCA9555_REGISTER reg;
    uint8_t pin_in_port;

    // Determine which port and pin within that port
    if (io_pin < 8) {
        reg = PCA9555_REG_CONFIGURATION_0;
        pin_in_port = io_pin;
    } else if (io_pin < 16) {
        reg = PCA9555_REG_CONFIGURATION_1;
        pin_in_port = io_pin - 8;
    } else {
        return ESP_ERR_INVALID_ARG; // Invalid pin
    }

    esp_err_t status = read_pca9555_reg(io_exp, reg, &port);
    if (status != ESP_OK)
        return status;

    port = (properties != PCA9555_OUTPUT) ? (port | (1 << pin_in_port)) : (port & ~(1 << pin_in_port));

    return write_pca9555_reg(io_exp, reg, port);
}

esp_err_t set_pca9555_io_pin_output_state(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin, uint8_t state) {
    uint8_t port;
    PCA9555_REGISTER reg;
    uint8_t pin_in_port;

    // Determine which port and pin within that port
    if (io_pin < 8) {
        reg = PCA9555_REG_OUTPUT_PORT_0;
        pin_in_port = io_pin;
    } else if (io_pin < 16) {
        reg = PCA9555_REG_OUTPUT_PORT_1;
        pin_in_port = io_pin - 8;
    } else {
        return ESP_ERR_INVALID_ARG; // Invalid pin
    }

    esp_err_t status = read_pca9555_reg(io_exp, reg, &port);
    if (status != ESP_OK)
        return status;

    port = (state != 0) ? (port | (1 << pin_in_port)) : (port & ~(1 << pin_in_port));

    return write_pca9555_reg(io_exp, reg, port);
}

int16_t get_io_port_register_value(PCA9555_IO_EXP* io_exp, uint8_t port, uint8_t reg_type) {
    PCA9555_REGISTER reg;
    uint8_t value = 0;

    // Calculate the register based on port and reg_type
    if (port > 1 || reg_type > 3) {
        return PCA9555_ERROR; // Invalid port or register type
    }

    reg = (PCA9555_REGISTER)(port + (reg_type * 2));

    esp_err_t status = read_pca9555_reg(io_exp, reg, &value);
    if (status != ESP_OK) {
        return PCA9555_ERROR;
    }

    return value;
}

int16_t get_io_port_16Bit_register_value(PCA9555_IO_EXP* io_exp, uint8_t port, uint8_t reg_type) {
    // returns a 16-bit value from the specified port and register type
    if (port > 1 || reg_type > 3) {
        return PCA9555_ERROR; // Invalid port or register type
    }
    PCA9555_REGISTER reg0 = (PCA9555_REGISTER)(port + (reg_type * 2));
    PCA9555_REGISTER reg1 = (PCA9555_REGISTER)(reg0 +
                                               1); // Get the next register for the second port
    uint8_t value0 = 0, value1 = 0;
    esp_err_t status0 = read_pca9555_reg(io_exp, reg0, &value0);
    if (status0 != ESP_OK) {
        return PCA9555_ERROR;
    }
    esp_err_t status1 = read_pca9555_reg(io_exp, reg1, &value1);
    if (status1 != ESP_OK) {
        return PCA9555_ERROR;
    }
    // Combine the two 8-bit values into a 16-bit value
    return (value1 << 8) | value0; // Return as a 16
    // bit value
}

esp_err_t get_pca9555_direct() {
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

#ifdef __cplusplus
}
#endif