#include "ESP32S3ZeroSdCard.h"
#include "ESP32S3ZeroConstants.h"

#include <Tactility/Log.h>
#include <Tactility/hal/sdcard/SpiSdCardDevice.h>

using tt::hal::sdcard::SpiSdCardDevice;

std::shared_ptr<SdCardDevice> createESP32S3ZeroSdCard() {
    // Since the board uses software SPI for SD card, we need to set up the pins manually
    auto* configuration = new SpiSdCardDevice::Config(
        ESP32S3_ZERO_SD_PIN_CS,
        ESP32S3_ZERO_SD_PIN_MOSI,
        ESP32S3_ZERO_SD_PIN_MISO,
        ESP32S3_ZERO_SD_PIN_CLOCK,
        SdCardDevice::MountBehaviour::Anytime, // Changed to allow mounting anytime for better control
        std::make_shared<tt::Mutex>(),
        std::vector<gpio_num_t>(),
        SPI2_HOST // Use a different SPI host than the display
    );


    // Set additional configuration options
    configuration->spiFrequencyKhz = 25000000; // Lower SD card frequency for better compatibility

    auto* sdcard = (SdCardDevice*)new SpiSdCardDevice(
        std::unique_ptr<SpiSdCardDevice::Config>(configuration)
    );

    return std::shared_ptr<SdCardDevice>(sdcard);
}
