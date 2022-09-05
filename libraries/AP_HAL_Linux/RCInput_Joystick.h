#pragma once

#include "AP_HAL_Linux.h"
#include "RCInput.h"
#include <linux/joystick.h>
#include <vector>

namespace Linux {

class RCInput_Joystick : public RCInput
{
public:
    void init() override;
    void _timer_tick(void) override;
    RCInput_Joystick(const char * path_dev_input_js) : dev_input_js(path_dev_input_js) {}
    ~RCInput_Joystick();

private:
    bool opendev(void);
    uint32_t last_event_millis;
    std::vector<struct js_event> axes;
    std::vector<struct js_event> buttons;
    const char * dev_input_js = NULL;
    int fd = -1;
    void _process_js_axis(struct js_event & axis);
    void _process_js_button(struct js_event & button);

    void teardown() override;
};

}
