#include <AP_HAL/AP_HAL.h>

#include <fcntl.h>
#include <stdio.h>

#include "RCOutput_PWM.h"

#ifdef DEBUG
#define debug(fmt, args ...) do { fprintf(stderr,"[RCOutput_PWM]: %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

extern const AP_HAL::HAL& hal;

using namespace Linux;

void RCOutput_PWM::teardown()
{
    stop_pwm();
}

RCOutput_PWM::RCOutput_PWM(uint8_t channel_offset, uint8_t pwmchip, uint8_t pwmch):
    _frequency(50),
    _pulses_buffer(new uint16_t[pwmch - channel_offset]),
    _pwmchip(pwmchip),
    _pwmch(pwmch),
    _channel_offset(channel_offset)
{
debug("");
}

RCOutput_PWM::~RCOutput_PWM()
{
debug("");
}

void RCOutput_PWM::init()
{
    debug("");
    for( unsigned ch = 0; ch < _pwmch; ++ch ) {
      _pulses_buffer[ch] = 0;
    }

    /* Set the initial frequency */
    set_freq(0, _frequency);
}

static uint32_t hz2ns(uint16_t freq_hz)
{
return 1000*1000*1000 / freq_hz;
}

void RCOutput_PWM::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    debug("chmask=%u freq_hz=%u", chmask, freq_hz);
    /* Correctly finish last pulses */
    _frequency = freq_hz;
    for(unsigned ch = 0; ch < _pwmch; ++ch) {
        writePwmFile(ch, "period", hz2ns(freq_hz));
        write(ch, _pulses_buffer[ch]);
    }
}

uint16_t RCOutput_PWM::get_freq(uint8_t ch)
{
    debug("ch=%u", ch);
    return _frequency;
}

bool RCOutput_PWM::writePwmFile(unsigned ch, const char * subfile, uint32_t val)
{
char valstr[12];
char filename[100];
int n = snprintf(valstr,sizeof(valstr),"%i",val);

int fn = snprintf(filename,sizeof(filename),"/sys/class/pwm/pwmchip%u/pwm%u/%s", _pwmchip, ch, subfile);
if(fn < 0)
    {
    return false;
    }

int fd = ::open(filename, O_WRONLY);
if(fd < 0)
    {
    debug("failed to open %s", filename);
    return false;
    }

ssize_t wrote = ::write(fd, valstr, n);
::close(fd);
return wrote == n;
}

void RCOutput_PWM::enable_ch(uint8_t ch)
{
   if( ch < _pwmch) {
     debug("ch=%u", ch);
     writePwmFile(ch, "enable", 1);
  }
}

void RCOutput_PWM::disable_ch(uint8_t ch)
{
    if(ch < _pwmch) {
      debug("ch=%u", ch);
      writePwmFile(ch, "enable", 0);
      write(ch, 0);
    }
}


bool RCOutput_PWM::force_safety_on() {
    debug("");
    /* Shutdown before sleeping. */
    bool ret = true;
    for( unsigned ch = 0; ch < _pwmch; ++ch) {
      ret &= writePwmFile(ch, "enable", 0);
    }
    return ret;
}

void RCOutput_PWM::force_safety_off() {
    debug("");
    /* Restart the device and enable auto-incremented write */
    for( unsigned ch = 0; ch < _pwmch; ++ch) {
      writePwmFile(ch, "enable", 1);
    }
}

void RCOutput_PWM::cork()
{
    _corking = true;
}

void RCOutput_PWM::push()
{
    if (!_corking) {
        return;
    }
    _corking = false;

    for( unsigned ch = 0; ch < _pwmch; ++ch) {
      writePwmFile(ch, "duty_cycle", _pulses_buffer[ch] * 1000);
    }
}

void RCOutput_PWM::write(uint8_t ch, uint16_t period_us)
{
    if(ch >= _pwmch) {
        return;
    }

    debug("write(ch=%u, period_us=%u", ch, period_us);
    _pulses_buffer[ch] = period_us;
    /* linux API takes nanoseconds */
    if (!_corking) {
      _corking = true;
      push();
    }
}

uint16_t RCOutput_PWM::read(uint8_t ch)
{
    if(ch >= _pwmch) {
        return 0;
    }

    debug("read(ch=%u)=%u", ch, _pulses_buffer[ch]);
    return _pulses_buffer[ch];
}

void RCOutput_PWM::read(uint16_t* period_us, uint8_t len)
{
    debug("read(%u)", len);
    for (int i = 0; i < len; i++) {
        period_us[i] = read(0 + i);
    }
}
