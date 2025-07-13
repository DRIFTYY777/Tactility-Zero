#pragma once

#include <Tactility/TactilityCore.h>
#include <Tactility/hal/keyboard/KeyboardDevice.h>

class ESP32S3ZeroKeyboard : public tt::hal::keyboard::KeyboardDevice {

private:

    lv_indev_t* _Nullable deviceHandle = nullptr;

public:

    std::string getName() const final { return "ESP32S3-ZERO Keyboard"; }
    std::string getDescription() const final { return "I2C keyboard"; }

    bool start(lv_display_t* display) override;
    bool stop() override;
    bool isAttached() const override;
    lv_indev_t* _Nullable getLvglIndev() override { return deviceHandle; }
};

std::shared_ptr<tt::hal::keyboard::KeyboardDevice> createKeyboard();
