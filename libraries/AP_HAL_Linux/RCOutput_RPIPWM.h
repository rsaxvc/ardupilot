#pragma once

#include "AP_HAL_Linux.h"
#include "RCInput.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#define RCOUT_RPIPWM_CHN_NUM 2

#include "AP_HAL_Linux.h"

namespace Linux {

class RCOutput_RPIPWM : public AP_HAL::RCOutput {
public:
    RCOutput_RPIPWM(uint8_t channel_offset);
    ~RCOutput_RPIPWM();
    void     init() override;
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    bool     force_safety_on() override;
    void     force_safety_off() override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    bool     supports_gpio() override { return false; };
    void     teardown();

private:
    //Physical adresses of peripherals. Are different on different Raspberries.
    uint32_t pwm_base;

    //registers
    static volatile uint32_t *pwm_reg;

    void set_physical_addresses(void);
    void* map_peripheral(uint32_t base, uint32_t len);

    void reset();
    void write_raw(uint8_t ch, uint16_t period_us);
    void stop_pwm();

    uint16_t *_pulses_buffer;
    uint8_t _channel_offset;
};

}
