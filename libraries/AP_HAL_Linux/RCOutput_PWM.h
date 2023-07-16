#pragma once

#include "AP_HAL_Linux.h"
#include "AP_HAL/RCOutput.h"
#include <stdint.h>

namespace Linux {

class RCOutput_PWM : public AP_HAL::RCOutput {
public:
    RCOutput_PWM(uint8_t channel_offset, uint8_t pwmchip, uint8_t pwmch);
    ~RCOutput_PWM();
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;

    void     cork() override;
    void     push() override;

    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;

    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    bool     force_safety_on() override;
    void     force_safety_off() override;
    bool     supports_gpio() override { return false; };
    void     teardown();

private:
    bool writePwmFile(unsigned ch, const char * subfile, uint32_t val);
    bool _initialized;
    bool _corking;
    uint8_t _pwmchip;
    uint8_t _pwmch;
    void reset();
    void stop_pwm();

    uint16_t * _pulses_buffer;
    uint16_t _frequency;
    uint8_t _channel_offset;
};

}
