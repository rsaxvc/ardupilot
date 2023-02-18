#include "RCOutput_RPISMI.h"

#include "bcm2835_smi.h"

#include <cmath>
#include <dirent.h>
#include <fcntl.h>
#include <malloc.h>
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

#define PAGE_SIZE (4*1024)
#define WORDS_PER_PAGE (PAGE_SIZE/sizeof(uint16_t))
#define TICKS_PER_PAGE (PAGE_SIZE*8/16)
#define BITS_PER_DSHOT 16

//See https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
//See https://www.speedgoat.com/products/dshot

#define DSHOT150_NS 6667
#define DSHOT150_SAMPLE_NS 64
#define DSHOT150_T0H_NS 2500
#define DSHOT150_T0L_NS (DSHOT150_NS-DSHOT150_T0H_NS)
#define DSHOT150_T1H_NS 5000
#define DSHOT150_T1L_NS (DSHOT150_NS-DSHOT150_T0H_NS)

#define DSHOT300_NS 3333
#define DSHOT300_SAMPLE_NS 32
#define DSHOT300_T0H_NS 1500
#define DSHOT300_T0L_NS (DSHOT600_NS-DSHOT600_T0H_NS)
#define DSHOT300_T1H_NS 2500
#define DSHOT300_T1L_NS (DSHOT600_NS-DSHOT600_T0H_NS)

#define DSHOT600_NS 1667
#define DSHOT600_SAMPLE_NS 14
#define DSHOT600_T0H_NS 625
#define DSHOT600_T0L_NS (DSHOT600_NS-DSHOT600_T0H_NS)
#define DSHOT600_T1H_NS 1250
#define DSHOT600_T1L_NS (DSHOT600_NS-DSHOT600_T0H_NS)

#define DSHOT1200_NS 833
#define DSHOT1200_SAMPLE_NS 8
#define DSHOT1200_T0H_NS 313
#define DSHOT1200_T0L_NS (DSHOT1200_NS-DSHOT1200_T0H_NS)
#define DSHOT1200_T1H_NS 625
#define DSHOT1200_T1L_NS (DSHOT1200_NS-DSHOT1200_T0H_NS)

#define TICKS_PER_BIT 3
#define TICKS_PER_DSHOT (BITS_PER_DSHOT * TICKS_PER_BIT)

#define NORMAL_PERIOD_US (1000000 / 50)
extern const AP_HAL::HAL& hal;

/*
RCOutput RPI-SMI Theory of Operation

Raspberry PIs contain a parallel memory interface controller.
Ignoring clock/strobe signals, we use SMI to implement a 16-bit parallel bus,
where each bit implements a separate output channel.

Writing enough data to the SMI-chardev will use a DMA transfer with reliable timing.

Limitations:
    SMI-chardev DMA requires physically contiguous memory.
    This driver uses a single 4K-aligned allocation.

    Currently only 16-bit mode is used. 8-bit mode would allow more bits/page,
    but most of the low-bits are muxed with useful functions like SPI/I2C.

    To simplify the implementation, all channels must use the same output mode.
*/

RCOutput_RPISMI::RCOutput_RPISMI(uint8_t channel_offset):
    _frequency(50),
    _channel_offset(channel_offset)
{
}

RCOutput_RPISMI::~RCOutput_RPISMI()
{
    _thread_done_request = true;
    while(!_thread_done_response) {
        usleep(100);
    }
}

void RCOutput_RPISMI::init()
{
    reset_all_channels();

    /* Set the initial frequency */
    set_freq(0, 50);

    struct sched_param param = { .sched_priority = 15 };
    pthread_attr_t attr;

    /* Initialize thread and mutex */
    int ret;
    ret = pthread_mutex_init(&_mutex, nullptr);
    if (ret != 0) {
        perror("RCout_RPIPWM: failed to init mutex\n");
        return;
    }

    pthread_mutex_lock(&_mutex);

    ret = pthread_attr_init(&attr);
    if (ret != 0) {
        perror("RCOut_RPIPWM: failed to init attr\n");
        return;
    }
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    ret = pthread_create(&_thread, &attr, _control_thread, this);
    if (ret != 0) {
        perror("RCOut_RPIPWM: failed to create thread\n");
        return;
    }

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

void RCOutput_RPISMI::set_output_mode(uint32_t mask, enum output_mode new_mode)
{
    pthread_mutex_lock(&_mutex);
    mode = new_mode;
    pthread_mutex_unlock(&_mutex);
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
    if (ch >= (RPISMI_CHAN_COUNT - _channel_offset)) {
        return;
    }

    _request_period_us[ch] = period_us;

    if (!_corking) {
        _corking = true;
        push();
    }
}

void RCOutput_RPISMI::cork()
{
    _corking = true;
}

static uint16_t dshot_throttle(uint16_t period_us)
{
if( period_us < 1000 )
    return 48;
else if( period_us > 2047 )
    return 2047;
else
    return period_us;
}

static uint16_t dshot_frame(uint16_t period_us, bool telem_request)
{
const uint16_t crc_input = dshot_throttle(period_us) << 1 | telem_request;
const uint16_t crc = (crc_input ^ (crc_input >> 4) ^ (crc_input >> 8)) & 0x0F;
return crc_input << 4 | crc;
}

void RCOutput_RPISMI::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;

    pthread_mutex_lock(&_mutex);
    memcpy(_period_us, _request_period_us, sizeof(_period_us));
    pthread_mutex_unlock(&_mutex);
    memset(_request_period_us, 0, sizeof(_request_period_us));

    if (sem.take_nonblocking()) {
        return;
    }

    sem.give();
}

uint16_t RCOutput_RPISMI::read(uint8_t ch)
{
    if( ch < RPISMI_CHAN_COUNT ) {
        return _period_us[ch];
    } else {
        return 0;
    }
}

void RCOutput_RPISMI::read(uint16_t* period_us, uint8_t len)
{
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}

void RCOutput_RPISMI::set_dshot_esc_type(DshotEscType dshot_esc_type)
{
    _dshot_esc_type = dshot_esc_type;
    switch (_dshot_esc_type) {
        case DSHOT_ESC_BLHELI_S:
        case DSHOT_ESC_BLHELI_EDT_S:
            DSHOT_BIT_WIDTH_TICKS = DSHOT_BIT_WIDTH_TICKS_S;
            DSHOT_BIT_0_TICKS = DSHOT_BIT_0_TICKS_S;
            DSHOT_BIT_1_TICKS = DSHOT_BIT_1_TICKS_S;
            break;
        default:
            DSHOT_BIT_WIDTH_TICKS = DSHOT_BIT_WIDTH_TICKS_DEFAULT;
            DSHOT_BIT_0_TICKS = DSHOT_BIT_0_TICKS_DEFAULT;
            DSHOT_BIT_1_TICKS = DSHOT_BIT_1_TICKS_DEFAULT;
            break;
    }
}

static void set_output_mode(AP_HAL::RCOutput::output_mode new_mode, struct smi_settings & smi_config)
{
uint32_t sample_ns = 0;
switch(new_mode)
    {
    case AP_HAL::RCOutput::MODE_PWM_DSHOT150:
        sample_ns = DSHOT150_SAMPLE_NS;
        break;
    case AP_HAL::RCOutput::MODE_PWM_DSHOT300:
        sample_ns = DSHOT300_SAMPLE_NS;
        break;
    case AP_HAL::RCOutput::MODE_PWM_DSHOT600:
        sample_ns = DSHOT600_SAMPLE_NS;
        break;
    case AP_HAL::RCOutput::MODE_PWM_DSHOT1200:
        sample_ns = DSHOT1200_SAMPLE_NS;
        break;
    case AP_HAL::RCOutput::MODE_PWM_NORMAL:
        sample_ns = 1000000000 / 50 / WORDS_PER_PAGE;
        break;
    default:
        //Invalid mode chosen
        return;
    }

/*Change frequency here*/
smi_config.write_setup_time = 2*(sample_ns / 6);
smi_config.write_hold_time = smi_config.write_setup_time;
smi_config.write_strobe_time = sample_ns - smi_config.write_setup_time - smi_config.write_hold_time;
}


static void pack_dshot(uint16_t * buffer, const uint16_t * periods_us)
{
    uint16_t dshot_frames[RPISMI_CHAN_COUNT];
    for (unsigned ch = 0; ch < RPISMI_CHAN_COUNT; ch++) {
        dshot_frames[ch] = dshot_frame(periods_us[ch], 0);
    }

    for (unsigned i = 0, j = 0; i < TICKS_PER_DSHOT; i += TICKS_PER_BIT, j++ ) {
        buffer[i] = 0xFFFF;
        uint16_t bitslice = 0;
        for (unsigned ch = 0; ch < RPISMI_CHAN_COUNT; ch++) {
            bitslice |= !!(dshot_frames[ch] & (1<<j)) << ch;
        }
        buffer[i+1] = bitslice;
        buffer[i+2] = 0x0000;
    }
}

static void pack_normal_pwm(uint16_t * buffer, const uint16_t * periods_us)
{
    for (unsigned i = 0; i < WORDS_PER_PAGE; ++i ) {
	uint16_t word = 0;
	unsigned tick_us = i * NORMAL_PERIOD_US / WORDS_PER_PAGE;
	for (unsigned ch = 0; ch < RPISMI_CHAN_COUNT; ++ch){
            if(tick_us < periods_us[ch]){
		word |= 1 << ch;
            }
        buffer[i] = word;
	}
    }
}

void* RCOutput_RPISMI::_control_thread(void *arg) {
    RCOutput_RPISMI* rcout = (RCOutput_RPISMI *) arg;

    uint16_t * io_buffer = (uint16_t*)memalign(PAGE_SIZE, PAGE_SIZE);
    if(!io_buffer) {
        fprintf(stderr, "failed to allocate SMI DMA io buffer page\n");
        exit(-1);
    }

    int fd = open(dev_output_smi, O_RDWR);
    if(fd < 0)
        {
        fprintf(stderr, "failed to open %s\n", dev_output_smi);
        exit(-1);
        }

    struct smi_settings smi_config;

    int res = ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS, &smi_config);
    if(res < 0) {
        close(fd);
        fd = -1;
        fprintf(stderr, "ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS)=%i\n", res);
        exit(-1);
    }

    smi_config.data_width = SMI_WIDTH_16BIT;

    int32_t old_freq_hz = -1;
    enum output_mode old_pwm_mode = MODE_PWM_NONE;

    while (!rcout->_thread_done_request) {
        //Gather everything needed to output a batch of signals.
        pthread_mutex_lock(&rcout->_mutex);
        enum output_mode pwm_mode = rcout->mode;
        uint16_t freq_hz = rcout->_frequency;
        uint16_t period_us[RPISMI_CHAN_COUNT];
        memcpy(period_us, rcout->_period_us, sizeof(rcout->_period_us));
        pthread_mutex_unlock(&rcout->_mutex);

        if(old_freq_hz != freq_hz || old_pwm_mode != pwm_mode){
            //reconfigure here
            ::set_output_mode(pwm_mode, smi_config);
            old_freq_hz = freq_hz;
            old_pwm_mode = pwm_mode;
            res = ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &smi_config);
            if(res < 0)
            {
                close(fd);
                fd = -1;
                fprintf(stderr, "ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS)=%i\n", res);
                exit(-1);
            }
        }

        //Pack bitstreams according to mode
        switch (pwm_mode) {
            case MODE_PWM_NORMAL:
                pack_normal_pwm(io_buffer, period_us);
                break;
            default:
                pack_dshot(io_buffer, period_us);
                break;
        }

        //Transfer the bits out
        ::write(fd, io_buffer, PAGE_SIZE);
    }
    free(io_buffer);
    close(fd);
    rcout->_thread_done_response = true;
    return NULL;
}

