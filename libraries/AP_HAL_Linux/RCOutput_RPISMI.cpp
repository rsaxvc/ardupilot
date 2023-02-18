#include "RCOutput_RPISMI.h"

#include <cmath>
#include <dirent.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>

#include <sys/ioctl.h>
#include <AP_HAL/AP_HAL.h>

using namespace Linux;

#define PWM_CHAN_COUNT 16

extern const AP_HAL::HAL& hal;

RCOutput_RPISMI::RCOutput_RPISMI(  uint8_t channel_offset):
    _frequency(50),
    _pulses_buffer(new uint16_t[PWM_CHAN_COUNT - channel_offset]),
    _channel_offset(channel_offset)
{
fd = open(dev_output_smi, O_RDWR);
if(fd < 0)
    {
    fprintf(stderr, "failed to open %s\n", dev_output_smi);
    exit(-1);
    }
int res = ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS, &smi_config);
if(res < 0)
    {
    close(fd);
    fd = -1;
    fprintf(stderr, "ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS)=%i\n", res);
    exit(-1);
    }
smi_config.data_width = SMI_WIDTH_16BIT;
write_settings();
}

void RCOutput_RPISMI::write_settings()
{
int res = ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &smi_config);
if(res < 0)
    {
    close(fd);
    fd = -1;
    fprintf(stderr, "ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS)=%i\n", res);
    exit(-1);
    }
}

RCOutput_RPISMI::~RCOutput_RPISMI()
{
    close(fd);
    fd = -1;
    delete [] _pulses_buffer;
}

void RCOutput_RPISMI::init()
{
    reset_all_channels();

    /* Set the initial frequency */
    set_freq(0, 50);
}

void RCOutput_RPISMI::reset_all_channels()
{
    if (sem.take(10)) {
        return;
    }

    /* Wait for the last pulse to end */
    hal.scheduler->delay(2);

    sem.give();
}

void RCOutput_RPISMI::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (sem.take(10)) {
        return;
    }

    _frequency = freq_hz;

    smi_config.write_strobe_time = 1000000 / freq_hz;
    write_settings();

    /*Change frequency here*/
    sem.give();
}

uint16_t RCOutput_RPISMI::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_RPISMI::enable_ch(uint8_t ch)
{

}

void RCOutput_RPISMI::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

bool RCOutput_RPISMI::force_safety_on() {
    if (sem.take(10)) {
        return false;
    }
    /* Shutdown before sleeping. */
    sem.give();
    return true;
}

void RCOutput_RPISMI::force_safety_off() {
    if (sem.take(10)) {
        return;
    }
    /* Restart the device and enable auto-incremented write */
    sem.give();
}

void RCOutput_RPISMI::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= (PWM_CHAN_COUNT - _channel_offset)) {
        return;
    }
    _pulses_buffer[ch] = period_us;
    _pending_write_mask |= (1U << ch);

    if (!_corking) {
        _corking = true;
        push();
    }
}

void RCOutput_RPISMI::cork()
{
    _corking = true;
}

void RCOutput_RPISMI::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;

    if (_pending_write_mask == 0)
        return;

    static const unsigned BITS_PER_DSHOT = 16;
    static const unsigned TICKS_PER_BIT = 3;
    uint16_t data[BITS_PER_DSHOT * TICKS_PER_BIT] = {0};

    for (unsigned ch = 0; ch < PWM_CHAN_COUNT; ch++) {
        uint16_t period_us = _pulses_buffer[ch];
        for(unsigned bit = 0; bit < BITS_PER_DSHOT; bit++) {
            data[bit * TICKS_PER_BIT] |= 1<<ch;
            if(period_us & (1<<bit)) {
                data[bit * TICKS_PER_BIT+1] |= 1<<ch;
            }
        }
    }

    if (sem.take_nonblocking()) {
        return;
    }

    ::write(fd, &data, sizeof(data));
    _pending_write_mask = 0;
    sem.give();
}

uint16_t RCOutput_RPISMI::read(uint8_t ch)
{
    return _pulses_buffer[ch];
}

void RCOutput_RPISMI::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}
