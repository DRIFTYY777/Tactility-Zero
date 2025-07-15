#include "ESP32S3Zero.h"
#include "InitBoot.h"
#include "hal/ESP32S3ZeroConstants.h"
#include "hal/ESP32S3ZeroDisplay.h"
#include "hal/ESP32S3ZeroKeyboard.h"
#include "hal/ESP32S3ZeroSdCard.h"

#include <Tactility/Log.h>
#include <Tactility/lvgl/LvglSync.h>
#include <lvgl.h>

#define ESP32S3_ZERO_SPI_TRANSFER_SIZE_LIMIT (ESP32S3_ZERO_LCD_HORIZONTAL_RESOLUTION * ESP32S3_ZERO_LCD_VERTICAL_RESOLUTION * LV_COLOR_DEPTH / 8)

using namespace tt::hal;

extern const Configuration esp32s3_zero_config = {
    .initBoot = initBoot,
    .createDisplay = createDisplay,
    .createKeyboard = createKeyboard,
    .sdcard = createESP32S3ZeroSdCard(),
    .power = nullptr,
    .i2c = {
        // Internal I2C bus configuration (I2C_NUM_0) Keyboard
        i2c::Configuration {
            .name = "Internal",
            .port = I2C_NUM_0,
            .initMode = i2c::InitMode::ByTactility,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = ESP32S3_ZERO_I2C_SDA_0,
                .scl_io_num = ESP32S3_ZERO_I2C_SCL_0,
                .sda_pullup_en = false, // pulled up by the 4.7k resistors on the board
                .scl_pullup_en = false, // pulled up by the 4.7k resistors on the board
                .master = {
                    .clk_speed = 400000 // 400kHz
                },
               .clk_flags = 0
            }
        },
        // External I2C bus configuration (I2C_NUM_1)
        i2c::Configuration {
            .name = "External",
            .port = I2C_NUM_1,
            .initMode = i2c::InitMode::Disabled,
            .isMutable = true,
            .config = (i2c_config_t) {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = ESP32S3_ZERO_I2C_SDA_1,
                .scl_io_num = ESP32S3_ZERO_I2C_SDA_1,
                .sda_pullup_en = false,
                .scl_pullup_en = false,
                .master = {
                    .clk_speed = 400000
                },
                .clk_flags = 0
            }
        }
    },
    .spi = {
        // SPI configuration for the SD card
        spi::Configuration {
            .device = SPI2_HOST, 
            .dma = SPI_DMA_CH_AUTO, 
            .config = { 
                .mosi_io_num = ESP32S3_ZERO_SD_PIN_MOSI,
                .miso_io_num = ESP32S3_ZERO_SD_PIN_MISO,
                .sclk_io_num = ESP32S3_ZERO_SD_PIN_CLOCK,
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .data4_io_num = GPIO_NUM_NC,
                .data5_io_num = GPIO_NUM_NC,
                .data6_io_num = GPIO_NUM_NC,
                .data7_io_num = GPIO_NUM_NC,
                .data_io_default_level = false,
                .max_transfer_sz = 4192, // SD card transfers
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = nullptr
        },

        // SPI configuration for the display
        spi::Configuration {
            .device = SPI3_HOST,
            .dma = SPI_DMA_CH_AUTO,
            .config = {
                .mosi_io_num = ESP32S3_ZERO_LCD_PIN_MOSI,
                .miso_io_num = ESP32S3_ZERO_LCD_PIN_MISO,
                .sclk_io_num = ESP32S3_ZERO_LCD_PIN_CLOCK,
                .quadwp_io_num = GPIO_NUM_NC,
                .quadhd_io_num = GPIO_NUM_NC,
                .data4_io_num = GPIO_NUM_NC,
                .data5_io_num = GPIO_NUM_NC,
                .data6_io_num = GPIO_NUM_NC, 
                .data7_io_num = GPIO_NUM_NC,
                .data_io_default_level = false,
                .max_transfer_sz = ESP32S3_ZERO_SPI_TRANSFER_SIZE_LIMIT,
                .flags = 0,
                .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
                .intr_flags = 0
            },
            .initMode = tt::hal::spi::InitMode::ByTactility,
            .isMutable = false,
            .lock = tt::lvgl::getSyncLock() // esp_lvgl_port owns the lock for the display
        }
    },
    .uart = {}
};
