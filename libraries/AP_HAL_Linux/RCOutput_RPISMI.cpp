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

    Seamless DMA transfers require a single control descriptor. What this means
    to us in userspace is a single write() call to a subset of a single memory page.
*/

#define SMI_CLK 125e6
#define SMI_WRITE_STROBE_MASK 0x7F
#define SMI_BITS 16

#define DMA_MAX 2048 //Max physically contig from memalign()
#define DMA_BYTES_PER_CLOCK 2 //Always run SMI_BITS=16

/*For DSHOT:
    Each speed-mode has a fixed number of DMA-per-bit, then we tune the output
    clock divider (125MHz / (1 to 127)) to align the overall bit-length.

See https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
See https://www.speedgoat.com/products/dshot

DSHOT_: Generic ratios
DSHOT###_: speed-mode specific numbers
*/
#define DSHOT_BITS 16 //16 data bits per DSHOT frame

//how a single bit is sent. Either a short-pulse(zero-bit) or long-pulse(one-bit)
#define DSHOT_DMA_PER_BIT 16 //DMA Cycles per bit. must be < MASK, and DMA_PER_BIT * BITS must be < DMA_MAX
#define DSHOT_DMA_PER_ZRO 6  //Go high for # cycles, low for (DSHOT_DMA_PER_BIT - #) cycles
#define DSHOT_DMA_PER_ONE 12 //Go high for # cycles, low for (DSHOT_DMA_PER_BIT - #) cycle
#define DSHOT_DMA_COUNT (int)(DSHOT_DMA_PER_BIT * DSHOT_BITS) //Total number of DMA data accesses

//the relationship between speed-modes and clockage
#define DSHOT150_WRITE_STROBE_CYCLES 48
#define DSHOT300_WRITE_STROBE_CYCLES 24
#define DSHOT600_WRITE_STROBE_CYCLES 12
#define DSHOT1200_WRITE_STROBE_CYCLES 6

/*
For PWM/OneShot125: we set the DMA to run very slowly to generate the high-pulse,
    then sleep with the CPU for the low-pulse.
*/
#define RCPWM_WRITE_STROBE_CYCLES 125
#define RCPWM_MIN_US 1000
#define RCPWM_MAX_US 2000
#define RCPWM_MAX_DMA_COUNT 2048
#define RCPWM_HZ 50.0
#define RCPWM_TX_TIME_US (1e6/RCPWM_HZ)

#define OS125_WRITE_STROBE_CYCLES 125 //1MHz output rate, 1us per SMI tick
#define OS125_MIN_US 125//microseconds
#define OS125_MAX_US 250//microseconds
#define OS125_MAX_DMA_COUNT (OS125_MAX_US + 1)

#define MAX_DMA_COUNT 2048
static_assert(DSHOT_DMA_COUNT <= MAX_DMA_COUNT);
static_assert(OS125_MAX_DMA_COUNT <= MAX_DMA_COUNT);
static_assert(RCPWM_MAX_DMA_COUNT <= MAX_DMA_COUNT);

extern const AP_HAL::HAL& hal;

static uint16_t bitslice(unsigned bit, const uint16_t vals[SMI_BITS])
{
    //If this is a bottleneck, 8x8 bitboard rotation is much faster.
    uint16_t ret = 0;
    bit = 1 << bit;
    for (unsigned i = 0; i < SMI_BITS; ++i) {
        if (vals[i] & bit) {
            ret |= 1 << i;
        }
    }
    return ret;
}


static void wordset(uint16_t * output, uint16_t word, size_t count)
{
    while (count) {
        *output++ = word;
        count--;
    }
}

static uint16_t findSmallestGreaterVal(const uint16_t vals[SMI_BITS], uint16_t min)
{
    uint16_t best = 0xFFFF;

    for (int i = 0; i < SMI_BITS; ++i) {
        if (vals[i] > min) {
            best = std::min(best, vals[i]);
        }
    }
    return best;
}

static uint16_t packPwmWord(const uint16_t ticks[SMI_BITS], uint32_t limit)
{
    uint16_t ret = 0;
    for (int i = 0; i < SMI_BITS; ++i) {
        ret |= (uint16_t)(ticks[i] > limit) << i;
    }
    return ret;
}

RCOutput_RPISMI::RCOutput_RPISMI(uint8_t channel_offset):
    _frequency(50),
    _channel_offset(channel_offset)
{
}

RCOutput_RPISMI::~RCOutput_RPISMI()
{
    _thread_done_request = true;
    while (!_thread_done_response) {
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

bool RCOutput_RPISMI::force_safety_on()
{
    if (sem.take(10)) {
        return false;
    }
    /* Shutdown before sleeping. */
    sem.give();
    return true;
}

void RCOutput_RPISMI::force_safety_off()
{
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
    if ( ch < RPISMI_CHAN_COUNT ) {
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

void prefill_dshot(uint16_t * dma_buf)
{
    //Pre-fill the dma buffer for the static parts of the bits
    for (int bit = 0; bit < DSHOT_BITS; bit++) {
        //Locate the region for this bit
        uint16_t * bitBuf = dma_buf + DSHOT_DMA_PER_BIT * bit;

        //All DSHOT bits have this section high
        for (int dma = 0; dma < DSHOT_DMA_PER_ZRO; ++dma) {
            bitBuf[dma] = 0xFFFF;
        }
        //Skip over this part of each bit - it's where the zero/one distinction is made
        //All DSHOT bits have this section low
        for (int dma = DSHOT_DMA_PER_ONE; dma < DSHOT_DMA_PER_BIT; ++dma) {
            bitBuf[dma] = 0x0000;
        }
    }
}

static uint16_t dshot_throttle(uint16_t period_us)
{
    if ( period_us < 1000 ) {
        return 48;
    } else if ( period_us > 2047 ) {
        return 2047;
    } else {
        return period_us;
    }
}

static uint16_t dshot_frame(uint16_t period_us, bool telem_request)
{
    const uint16_t crc_input = dshot_throttle(period_us) << 1 | telem_request;
    const uint16_t crc4 = (crc_input ^ (crc_input >> 4) ^ (crc_input >> 8)) & 0x0F;
    return crc_input << 4 | crc4;
}

static size_t pack_dshot(uint16_t * dma_buf, const uint16_t * periods_us)
{
    uint16_t vals[SMI_BITS];
    for (int b = DSHOT_BITS - 1; b >= 0; b--) {
        vals[b] = dshot_frame(periods_us[b], false);
    }
    for (int b = DSHOT_BITS - 1; b >= 0; b--) {
        uint16_t * bitBuf = dma_buf + DSHOT_DMA_PER_BIT * b;
        uint16_t slice = bitslice(b, vals);
        wordset(bitBuf + DSHOT_DMA_PER_ZRO, slice, DSHOT_DMA_PER_ONE - DSHOT_DMA_PER_ZRO);
    }
    return DSHOT_DMA_PER_BIT * DSHOT_BITS;
}

static size_t pack_normal_pwm(uint16_t * dma_buf, const uint16_t * periods_us)
{
    uint32_t maxDmaTicks = RCPWM_MAX_DMA_COUNT;
    uint32_t dmaTick = 0;
    while (dmaTick < maxDmaTicks) {
        uint16_t next = std::min((uint16_t)RCPWM_MAX_DMA_COUNT, findSmallestGreaterVal(periods_us, dmaTick));
        uint16_t word = packPwmWord(periods_us, dmaTick);
        wordset(dma_buf + dmaTick, word, next - dmaTick);
        dmaTick = next;
    }
    return RCPWM_MAX_DMA_COUNT;
}

void* RCOutput_RPISMI::_control_thread(void *arg)
{
    RCOutput_RPISMI* rcout = (RCOutput_RPISMI *) arg;

    uint16_t * dma_buf = (uint16_t*)memalign(4096, MAX_DMA_COUNT * DMA_BYTES_PER_CLOCK);
    if (!dma_buf) {
        fprintf(stderr, "failed to allocate SMI DMA io buffer page\n");
        exit(-1);
    }

    //Verify we are not crossing a page boundary
    static_assert(MAX_DMA_COUNT * DMA_BYTES_PER_CLOCK <= 4096);

    static constexpr char dev_output_smi[] = "/dev/smi";
    int fd = open(dev_output_smi, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "failed to open %s\n", dev_output_smi);
        exit(-1);
    }

    struct smi_settings smi_config;

    int res = ioctl(fd, BCM2835_SMI_IOC_GET_SETTINGS, &smi_config);
    if (res < 0) {
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

        if (old_freq_hz != freq_hz || old_pwm_mode != pwm_mode) {
            //reconfigure bus timing and pre-fill constant data as needed
            old_freq_hz = freq_hz;
            old_pwm_mode = pwm_mode;
            switch (pwm_mode) {
            case AP_HAL::RCOutput::MODE_PWM_DSHOT150:
                smi_config.write_strobe_time = DSHOT150_WRITE_STROBE_CYCLES;
                prefill_dshot(dma_buf);
                break;
            case AP_HAL::RCOutput::MODE_PWM_DSHOT300:
                smi_config.write_strobe_time = DSHOT300_WRITE_STROBE_CYCLES;
                prefill_dshot(dma_buf);
                break;
            case AP_HAL::RCOutput::MODE_PWM_DSHOT600:
                smi_config.write_strobe_time = DSHOT600_WRITE_STROBE_CYCLES;
                prefill_dshot(dma_buf);
                break;
            case AP_HAL::RCOutput::MODE_PWM_DSHOT1200:
                smi_config.write_strobe_time = DSHOT1200_WRITE_STROBE_CYCLES;
                prefill_dshot(dma_buf);
                break;

            case AP_HAL::RCOutput::MODE_PWM_NORMAL:
                smi_config.write_strobe_time = RCPWM_WRITE_STROBE_CYCLES;
                break;
            case AP_HAL::RCOutput::MODE_PWM_ONESHOT:
                smi_config.write_strobe_time = RCPWM_WRITE_STROBE_CYCLES;
                break;
            case AP_HAL::RCOutput::MODE_PWM_ONESHOT125:
                smi_config.write_strobe_time = OS125_WRITE_STROBE_CYCLES;
                break;

            default:
                //Invalid mode chosen
                fprintf(stderr, "illegal mode chosen:%i\n", (int)pwm_mode);
                continue;
            }

            res = ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS, &smi_config);
            if (res < 0) {
                close(fd);
                fd = -1;
                fprintf(stderr, "ioctl(fd, BCM2835_SMI_IOC_WRITE_SETTINGS)=%i\n", res);
                exit(-1);
            }
        }

        //Pack bitstreams according to mode
        size_t wrsz = 0;
        switch (pwm_mode) {
        case MODE_PWM_NORMAL:
            wrsz = pack_normal_pwm(dma_buf, period_us);
            break;
        case MODE_PWM_DSHOT150:
        case MODE_PWM_DSHOT300:
        case MODE_PWM_DSHOT600:
        case MODE_PWM_DSHOT1200:
            wrsz = pack_dshot(dma_buf, period_us);
            break;
        default:
            break;
        }

        //Transfer the bits out
        ::write(fd, dma_buf, wrsz * DMA_BYTES_PER_CLOCK);
    }
    free(dma_buf);
    close(fd);
    rcout->_thread_done_response = true;
    return NULL;
}

