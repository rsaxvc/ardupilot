#include <AP_HAL/AP_HAL.h>

#include <AP_HAL/system.h>
#include "RCInput_Joystick.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>

#ifdef DEBUG
#define debug(fmt, args ...) do { fprintf(stderr,"[RCInput_Joystick]: %s:%d: " fmt, __FUNCTION__, __LINE__, ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

// RC Input is consistently sent, and a lack of input received implies a loss of connection
// Linux Joystick API only sends initial state and changes of state.
//
// If EVENT_TIMEOUT milliseconds passes without an update from the joystick device,
// this driver will repeat the current joystick state as RCInput. Loss of joystick link
// is handled explicitly with ENODEV, which will also stop repeating these updates.
#define EVENT_TIMEOUT 300

extern const AP_HAL::HAL& hal;

// output range: 700usec ~ 2300usec
static const int16_t out_min = 1000;
static const int16_t out_max = 2000;
static const int16_t out_center = (out_max + out_min)/2;
static const int16_t out_halfspan = (out_max - out_min)/2;

using namespace Linux;

RCInput_Joystick::~RCInput_Joystick()
{
}

void RCInput_Joystick::teardown()
{
    if(fd >= 0)
    {
        close(fd);
        fd = -1;
    }
}

bool RCInput_Joystick::opendev()
{
    uint8_t num_axes = 0;
    uint8_t num_buttons = 0;

    if(!dev_input_js) {
        return false;
    }

    fd = ::open(dev_input_js, O_RDONLY | O_NONBLOCK);

    if (fd < 0) {
        perror("Could not open joystick file");
        return false;
    }

    if (ioctl(fd, JSIOCGAXES, &num_axes) < 0 ) {
        perror("JSIOCGAXES failed");
        goto failure_close_fd;
    }

    if (ioctl(fd, JSIOCGBUTTONS, &num_buttons) < 0 ) {
        perror("JSIOCGBUTTONS filed");
        goto failure_close_fd;
    }

    axes.resize(num_axes);
    buttons.resize(num_buttons);
    set_num_channels(num_axes+num_buttons);
    return true;

failure_close_fd:
    ::close(fd);
    fd = -1;
    return false;
}

void RCInput_Joystick::init()
{
    opendev();
}

// Process joystick measurement
void RCInput_Joystick::_process_js_axis(struct js_event & axis)
{
    // input range: -(2^15-1) ~ (2^15-1)
    // from https://www.kernel.org/doc/Documentation/input/joystick-api.txt
    const int16_t in_halfspan = 32767;

    const uint16_t out_value = out_center + (int32_t)out_halfspan * axis.value / in_halfspan;
    const uint16_t rc_channel = axis.number;
    debug("rc %u, axis %u: %i -> %u\n", rc_channel, axis.number, axis.value, out_value);
    _process_pwm_pulse( rc_channel, 0, out_value);
}

// Process button measurement
void RCInput_Joystick::_process_js_button(struct js_event & button)
{
    const uint16_t out_value = button.value ? out_max : out_min;
    const uint16_t rc_channel = axes.size() + button.number;
    debug("rc %u, button %u: %u -> %u\n", rc_channel, button.number, button.value, out_value);
    _process_pwm_pulse( rc_channel, 0, out_value);
}

void RCInput_Joystick::_timer_tick()
{
    if (fd < 0 && !opendev() ) {
        return;
    }

    uint32_t now_millis = AP_HAL::millis();

    // Process any pending joystick events
    struct js_event event;
    while( sizeof(event) == ::read((int)fd, &event, (size_t)sizeof(event))) {
        last_event_millis = now_millis;

        switch (event.type & ~JS_EVENT_INIT) {
            case JS_EVENT_AXIS:
                if( event.number < axes.size() ) {
                    axes[event.number] = event;
                    _process_js_axis(event);
                } else {
                    debug("Invalid axis %u value %i\n", event.number, (int)event.value);
                }
                break;
            case JS_EVENT_BUTTON:
                if( event.number < buttons.size() ) {
                    buttons[event.number] = event;
                    _process_js_button(event);
                } else {
                    debug("Invalid button %u value %u = %s\n", event.number, event.value, event.value ? "down" : "up");
                }
                break;
            default:
                /* Ignore other events. */
                break;
        }
    }

    // Check for joystick failure and close if needed
    if( errno == EAGAIN ) {
        // no error: for non-blocking API, just means all pended events were processed
    } else {
        // error: do not expect recovery, close joystick and exit
        switch( errno ) {
            case ENODEV:
                debug("joystick connection lost\n");
                break;
            default:
                debug("errno: %i\n", errno );
                break;
        }
        close(fd);
        fd = -1;
        return;
    }

    // handle lack of joystick updates
    if(now_millis - last_event_millis > EVENT_TIMEOUT) {
        debug("timeout waiting for new events, repeating current event state\n");
        for (auto &axis : axes) {
            _process_js_axis(axis);
        }
        for (auto &button : buttons) {
            _process_js_button(button);
        }
        last_event_millis = now_millis;
    }

}
