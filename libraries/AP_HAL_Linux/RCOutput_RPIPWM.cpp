#include <AP_HAL/AP_HAL.h>

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

#define DEBUG
#ifdef DEBUG
#define debug(fmt, args ...) do { fprintf(stderr,"[RCOutput_RPIPWM]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace Linux;

volatile uint32_t *RCOutput_RPIPWM::pwm_reg;
//See bcm2711-peripherals.pdf Chapter 8.6
#define RCOUT_RPIPWM_RPI01_PWM_BASE 0x2000C000
#define RCOUT_RPIPWM_RPI23_PWM_BASE 0x3F00C000
#define RCOUT_RPIPWM_RPI4_PWM_BASE 0xFE00C000
#define RCOUT_RPIPWM_PWM_LEN 4096

// Physical addresses of peripheral depends on Raspberry Pi's version
void RCOutput_RPIPWM::set_physical_addresses()
{
    const LINUX_BOARD_TYPE rpi_version = UtilRPI::from(hal.util)->detect_linux_board_type();

    if(rpi_version == LINUX_BOARD_TYPE::RPI_ZERO_1) {
        pwm_base = RCOUT_RPIPWM_RPI01_PWM_BASE;
    } else if (rpi_version == LINUX_BOARD_TYPE::RPI_2_3_ZERO2) {
        pwm_base = RCOUT_RPIPWM_RPI23_PWM_BASE;
    } else if (rpi_version == LINUX_BOARD_TYPE::RPI_4) {
        pwm_base = RCOUT_RPIPWM_RPI4_PWM_BASE;
    } else {
        AP_HAL::panic("Unknown rpi_version, cannot locate peripheral base address");
        return;
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

RCOutput_RPIPWM::RCOutput_RPIPWM(uint8_t channel_offset):
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
    set_physical_addresses();
    pwm_reg = (uint32_t *)map_peripheral(pwm_base, RCOUT_RPIPWM_PWM_LEN);
    //_initialized = true;

    reset_all_channels();

    /* Set the initial frequency */
    set_freq(0, 50);
}

void RCOutput_RPIPWM::reset_all_channels()
{
    /* do the thing */

    /* Wait for the last pulse to end */
    hal.scheduler->delay(2);
}

void RCOutput_RPIPWM::set_freq(uint32_t chmask, uint16_t freq_hz)
{

    /* Correctly finish last pulses */
    for (int i = 0; i < (RCOUT_RPIPWM_CHN_NUM - _channel_offset); i++) {
        write(i, _pulses_buffer[i]);
    }

    /* do the thing */
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
    /* Shutdown before sleeping. */
    //do the thing

    return true;
}

void RCOutput_RPIPWM::force_safety_off() {
    /* Restart the device and enable auto-incremented write */
    //do the thing
}

void RCOutput_RPIPWM::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= (RCOUT_RPIPWM_CHN_NUM - _channel_offset)) {
        return;
    }
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
