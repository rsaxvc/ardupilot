#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RSAXVC_V1
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

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

#include "GPIO.h"
#include "RCOutput_RPIPWM.h"
#include "Util_RPI.h"

#ifdef DEBUG
#define debug(fmt, args ...) do { fprintf(stderr,"[RCOutput_RPIPWM]: %s:%d: " fmt, __FUNCTION__, __LINE__, ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace Linux;

volatile uint32_t *RCOutput_RPIPWM::pwm_reg;
//See bcm2711-peripherals.pdf Chapter 8.6
#define RCOUT_RPIPWM_RPI4_PWM_BASE 0xFE00C000
#define RCOUT_RPIPWM_PWM_LEN 4096

// Physical addresses of peripheral depends on Raspberry Pi's version
void RCOutput_RPIPWM::set_physical_addresses()
{
    int version = UtilRPI::from(hal.util)->get_rpi_version();
    switch( version ) {
#if 0
        case 0:
            // 1 & zero are the same
            pwm_base = RCOUT_RPIPWM_RPI1_PWM_BASE;
            break;
        case 1:
        case 2:
            // 2 & 3 are the same
            pwm_base = RCOUT_RPIPWM_RPI2_PWM_BASE;
            break;
#endif
        case 3:
            pwm_base = RCOUT_RPIPWM_RPI4_PWM_BASE;
            break;
        default:
            fprintf(stderr,"unknown RPI version=%i\n", version);
            exit(-1);
            break;
    }
}

// Map peripheral to virtual memory
void *RCOutput_RPIPWM::map_peripheral(uint32_t base, uint32_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_CLOEXEC);
    void *vaddr;

    if (fd < 0) {
        printf("Failed to open /dev/mem: %m\n");
        printf("Make sure that CONFIG_STRICT_DEVMEM is disabled\n");
        return nullptr;
    }
    vaddr = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
    if (vaddr == MAP_FAILED) {
        printf("rpio-pwm: Failed to map peripheral at 0x%08x: %m\n", base);
        exit(-1);
    }

    close(fd);
    return vaddr;
}

void RCOutput_RPIPWM::teardown()
{
    stop_pwm();
}

void RCOutput_RPIPWM::init()
{
    uint64_t signal_states(0);

    set_physical_addresses();
    pwm_reg = (uint32_t *)map_peripheral(pwm_base, RCOUT_RPIPWM_PWM_LEN);
    _initialized = true;
}

// Processing signal
void RCOutput_RPIPWM::_timer_tick()
{
    uint32_t counter = 0;
    uint64_t signal_states(0);

    if (!_initialized) {
        return;
    }
}

#endif // CONFIG_HAL_BOARD_SUBTYPE


RCOutput_RPIPWM::RCOutput_RPIPWM(uint8_t channel_offset)
    _dev(std::move(dev)),
    _frequency(50),
    _pulses_buffer(new uint16_t[RCOUT_RPIPWM_CHN_NUM - channel_offset]),
    _channel_offset(channel_offset)
{
}

RCOutput_RPIPWM::~RCOutput_RPIPWM()
{
    delete [] _pulses_buffer;
}

void RCOutput_RPIPWM::init()
{
    reset_all_channels();

    /* Set the initial frequency */
    set_freq(0, 50);

    /* Enable RPIPWM PWM */
    if (_oe_pin_number != -1) {
        _enable_pin = hal.gpio->channel(_oe_pin_number);
        _enable_pin->mode(HAL_GPIO_OUTPUT);
        _enable_pin->write(0);
    }
}

void RCOutput_RPIPWM::reset_all_channels()
{
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }

    uint8_t data[] = {RPIPWM_RA_ALL_LED_ON_L, 0, 0, 0, 0};
    _dev->transfer(data, sizeof(data), nullptr, 0);

    /* Wait for the last pulse to end */
    hal.scheduler->delay(2);

    _dev->get_semaphore()->give();
}

void RCOutput_RPIPWM::set_freq(uint32_t chmask, uint16_t freq_hz)
{

    /* Correctly finish last pulses */
    for (int i = 0; i < (RCOUT_RPIPWM_CHN_NUM - _channel_offset); i++) {
        write(i, _pulses_buffer[i]);
    }

    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }

    _dev->get_semaphore()->give();
}

uint16_t RCOutput_RPIPWM::get_freq(uint8_t ch)
{
    return _frequency;
}

void RCOutput_RPIPWM::enable_ch(uint8_t ch)
{

}

void RCOutput_RPIPWM::disable_ch(uint8_t ch)
{
    write(ch, 0);
}

bool RCOutput_RPIPWM::force_safety_on() {
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return false;
    }
    /* Shutdown before sleeping. */
    _dev->write_register(RPIPWM_RA_ALL_LED_OFF_H, RPIPWM_ALL_LED_OFF_H_SHUT);

    _dev->get_semaphore()->give();
    return true;
}

void RCOutput_RPIPWM::force_safety_off() {
    if (!_dev || !_dev->get_semaphore()->take(10)) {
        return;
    }
    /* Restart the device and enable auto-incremented write */
    _dev->write_register(RPIPWM_RA_MODE1,
                         RPIPWM_MODE1_RESTART_BIT | RPIPWM_MODE1_AI_BIT);
    _dev->get_semaphore()->give();
}

void RCOutput_RPIPWM::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= (RCOUT_RPIPWM_CHN_NUM - _channel_offset)) {
        return;
    }
    if (_is_gpio_mask & (1U << ch)) {
        return;
    }
    write_raw(ch, period_us);
}

void RCOutput_RPIPWM::write_gpio(uint8_t chan, bool active)
{
    if (chan >= (RCOUT_RPIPWM_CHN_NUM - _channel_offset)) {
        return;
    }
    _is_gpio_mask |= (1U << chan);
    write_raw(chan, active);
}

void RCOutput_RPIPWM::write_raw(uint8_t ch, uint16_t period_us) {
    /* Common code used by both write() and write_gpio() */
    _pulses_buffer[ch] = period_us;
}

uint16_t RCOutput_RPIPWM::read(uint8_t ch)
{
    return _pulses_buffer[ch];
}

void RCOutput_RPIPWM::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}
