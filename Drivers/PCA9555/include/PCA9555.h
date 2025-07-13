#ifndef PCA9555_H
#define PCA9555_H

#include <driver/gpio.h>
#include <driver/i2c.h>

/* Error code definitions */
#define PCA9555_ERROR -1

#ifdef __cplusplus
extern "C" {
#endif

/**
     * @brief PCA9555 IO Pins mapping for all 16 pins
     */
typedef enum {
    PCA9555_IO0,
    PCA9555_IO1,
    PCA9555_IO2,
    PCA9555_IO3,
    PCA9555_IO4,
    PCA9555_IO5,
    PCA9555_IO6,
    PCA9555_IO7,
    PCA9555_IO8,
    PCA9555_IO9,
    PCA9555_IO10,
    PCA9555_IO11,
    PCA9555_IO12,
    PCA9555_IO13,
    PCA9555_IO14,
    PCA9555_IO15
} PCA9555_PINS;

/**
     * @brief PCA9555 port direction
     */
typedef enum {
    PCA9555_OUTPUT,
    PCA9555_INPUT
} PCA9555_PORT_DIRECTION;

/**
     * @brief PCA9555 initialization parameters
     */
typedef struct
{
    uint8_t I2C_ADDR;
    int i2c_master_port;
    // Only when mode is set to interrupt, otherwise it won't be used..
    gpio_num_t interrupt_pin;
    TaskHandle_t* interrupt_task;
} PCA9555_IO_EXP;

/**
     * @brief Setup interrupts using the builtin INT pin of the PCA9555 and interrupt handler+task
     * @param io_exp which contains the gpio pin where INT is connected (interrupt_pin)
     *               And optionally contains the task to run when interrupt triggered (interrupt_task) if not defined the
     *               default handler will be used.
     */
void setup_pca9555_interrupt_handler(PCA9555_IO_EXP* io_exp);

/**
     * @brief Get the current input state of the specified input pin (1 or 0)
     *
     * @param io_exp The io expander instance to read from or write to
     * @param io_pin The io expander pin to read the state from
     *
     * @return
     *     - 0   Success! Pin is Low
     *     - 1   Success! Pin is High
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the io expander
     */
int16_t get_io_pin_input_status(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin);

/**
     * @brief Get the current input state of all the io expander pins
     *
     * @param io_exp The io expander instance to read from or write to
     *
     * @return
     *     - 0x0000 - 0xFFFF  Success! Dump of input registers (16-bit value)
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the io expander
     */
int16_t get_pca9555_all_io_pin_input_status(PCA9555_IO_EXP* io_exp);

/**
     * @brief Get the current direction of all the io expander pins
     *
     * @param io_exp The io expander instance to read from or write to
     *
     * @return
     *     - 0x0000 - 0xFFFF  Success! Dump of configuration registers (16-bit value)
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the io expander
     */
int16_t get_all_io_pin_direction(PCA9555_IO_EXP* io_exp);

/**
     * @brief Get the current polarity inversion state of all the io expander pins
     *
     * @param io_exp The io expander instance to read from or write to
     *
     * @return
     *     - 0x0000 - 0xFFFF  Success! Dump of polarity inversion registers (16-bit value)
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the io expander
     */
int16_t get_all_io_polarity_inversion(PCA9555_IO_EXP* io_exp);

/**
     * @brief Get the current polarity inversion state of the specified io expander pin
     *
     * @param io_exp The io expander instance to read from or write to
     * @param io_pin The io expander pin to read polarity inversion from
     *
     * @return
     *     - 0   Success! Pin is Not inverted
     *     - 1   Success! Pin is Inverted
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the io expander
     */
int16_t get_io_pin_polarity_inversion(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin);

/**
     * @brief Get the current direction of the specified physical pin (0 means OUTPUT or 1 means INPUT)
     *
     * @param io_exp The io expander instance to read from or write to
     * @param io_pin The io expander pin to read the state from
     *
     * @return
     *     - 0   Success! Pin is OUTPUT
     *     - 1   Success! Pin is INPUT
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the io expander
     */
int16_t get_io_pin_direction(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin);

/**
     * @brief Sets all physical pins of the io expander to a specified direction (INPUT or OUTPUT)
     *
     * @param io_exp The io expander instance to read from or write to
     * @param properties The pin direction to be set (INPUT or OUTPUT)
     *
     * @return
     *     - ESP_OK  Success!
     *     - ESP_ERR Error!
     */
esp_err_t set_all_pca9555_io_pins_direction(PCA9555_IO_EXP* io_exp, PCA9555_PORT_DIRECTION properties);

/**
     * @brief Set physical pin of the io expander to a specified direction (INPUT or OUTPUT)
     *
     * @param io_exp The io expander instance to read from or write to
     * @param io_pin The io expander physical pin to be set
     * @param properties The pin direction to be set (INPUT or OUTPUT)
     *
     * @return
     *     - ESP_OK  Success!
     *     - ESP_ERR Error!
     */
esp_err_t set_pca9555_io_pin_direction(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin, PCA9555_PORT_DIRECTION properties);

/**
     * @brief Set physical pin of the io expander to an specified output state (HIGH(1) or LOW(0))
     *
     * @param io_exp The io expander instance to read from or write to
     * @param io_pin The io expander physical pin to be set
     * @param state The pin state to be set (1 or 0)
     *
     * @return
     *     - ESP_OK  Success!
     *     - ESP_ERR Error!
     *
     * @note Pin output state can be inverted with the inversion register
     */
esp_err_t set_pca9555_io_pin_output_state(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin, uint8_t state);

/**
     * @brief Set physical pin of the io expander to an specified output state (HIGH(1) or LOW(0))
     *
     * @param io_exp The io expander instance to read from or write to
     * @param io_pin The io expander physical pin to be set
     * @param state The pin state to be set (1 or 0)
     *
     * @return
     *     - ESP_OK  Success!
     *     - ESP_ERR Error!
     *
     * @note Pin output state can be inverted with the inversion register
     */
esp_err_t set_pca9555_io_pin_output_state(PCA9555_IO_EXP* io_exp, PCA9555_PINS io_pin, uint8_t state);

/**
     * @brief Read a value from a specific IO port register
     *
     * @param io_exp The io expander instance to read from
     * @param port Port number (0 or 1)
     * @param reg_type Register type (0=INPUT, 1=OUTPUT, 2=POLARITY_INV, 3=CONFIG)
     *
     * @return
     *     - 0x00 - 0xFF  Success! Value read from the register
     *     - PCA9555_ERROR(-1)  Error!   Something went wrong in the process of reading the register
     */
int16_t get_io_port_register_value(PCA9555_IO_EXP* io_exp, uint8_t port, uint8_t reg_type);


int16_t get_io_port_16Bit_register_value(PCA9555_IO_EXP* io_exp, uint8_t port, uint8_t reg_type);

static esp_err_t get_pca9555_direct();


#ifdef __cplusplus
}
#endif

#endif /* PCA9555_H */
