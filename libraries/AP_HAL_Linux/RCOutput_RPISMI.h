#pragma once

#include "AP_HAL_Linux.h"

#include "bcm2835_smi.h"

#define RPISMI_CHAN_COUNT 16
namespace Linux {

class RCOutput_RPISMI : public AP_HAL::RCOutput {
public:
    RCOutput_RPISMI(uint8_t channel_offset);

    ~RCOutput_RPISMI();
    void     init() override;
    void     reset_all_channels();
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    bool     force_safety_on() override;
    void     force_safety_off() override;
    void     write(uint8_t ch, uint16_t period_us) override;
    void     cork() override;
    void     push() override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    bool     supports_gpio() override { return false; };
    void set_dshot_esc_type(DshotEscType dshot_esc_type) override;
    void    set_output_mode(uint32_t mask, enum output_mode new_mode) override;
private:
    Semaphore sem;
    enum output_mode mode = MODE_PWM_DSHOT150;
    static constexpr char const * dev_output_smi = "/dev/smi";
    int fd = -1;
    struct smi_settings smi_config;
    unsigned _pending_write_mask;

    void reset();
    void write_settings();

    uint16_t _frequency;

    uint16_t _request_period_us[RPISMI_CHAN_COUNT];
    uint16_t _period_us[RPISMI_CHAN_COUNT];
    uint16_t *_io_buffer;

    bool _corking = false;
    uint8_t _channel_offset;

    DshotEscType _dshot_esc_type;

    void configure_timing(uint32_t sample_ns);

    /* thread related members */
    pthread_t _thread;
    pthread_mutex_t _mutex;
    pthread_cond_t _cond;
    void _run_rcout();
    static void *_control_thread(void *arg);
};

}
