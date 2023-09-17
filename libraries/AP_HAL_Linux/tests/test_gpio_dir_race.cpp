/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <pthread.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/GPIO.h>

using namespace Linux;

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

#define TEST_PIN 12

class TestGpioMode1 {
public:
    TestGpioMode1() { }

    bool run() {
        hal.gpio->pinMode(TEST_PIN, HAL_GPIO_INPUT);
        hal.gpio->pinModeValidate(TEST_PIN, HAL_GPIO_INPUT);
        hal.gpio->pinMode(TEST_PIN, HAL_GPIO_OUTPUT);
        hal.gpio->pinModeValidate(TEST_PIN, HAL_GPIO_OUTPUT);
        return true;
    }

protected:
};

TEST(LinuxGpio, override_run)
{
    TestGpioMode1 gpio;
    fprintf(stderr,"Initializing HAL\n");
    hal.gpio->init();
    fprintf(stderr,"Launching test run\n");
    while (gpio.run()) {
    }
}

AP_GTEST_MAIN()
