#include "ESP32S3ZeroDisplay.h"
#include "ESP32S3ZeroConstants.h"

#include <PwmBacklight.h>
#include <St7789Display.h>
#include <Tactility/Log.h>

#include <driver/spi_master.h>

#define TAG "esp32s3_zero_display"

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    // No touch device for this board, just the display
    std::shared_ptr<tt::hal::touch::TouchDevice> touch = nullptr;

    auto configuration = std::make_unique<St7789Display::Configuration>(
        (esp_lcd_spi_bus_handle_t)ESP32S3_ZERO_LCD_SPI_HOST,
        ESP32S3_ZERO_LCD_PIN_CS,
        ESP32S3_ZERO_LCD_PIN_DC,
        ESP32S3_ZERO_LCD_HORIZONTAL_RESOLUTION,
        ESP32S3_ZERO_LCD_VERTICAL_RESOLUTION,
        touch,  // No touch device
        true,   // swapXY - Enable for landscape rotation
        false,  // mirrorX - No horizontal mirror
        true,   // mirrorY - No vertical mirror
        false   // invertColor
    );

    // Set reset pin
    configuration->resetPin = ESP32S3_ZERO_LCD_PIN_RESET;

    // Configure backlight function
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    return std::make_shared<St7789Display>(std::move(configuration));
}
