/*
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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit.
 *
 * This file exists to avoid a namespace clash between CMSIS's arm_status
 * return value and uavcan, which is common enough here to be an issue.
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>
#include "HAL_ChibiOS_Class.h"

#if HAL_WITH_DSP
#include "DSP.h"
static ChibiOS::DSP dspDriver;
#else
#include "AP_HAL_Empty/DSP.h"
static Empty::DSP dspDriver;
#endif

AP_HAL::DSP* HAL_ChibiOS::getDspBackend()
{
return &dspDriver;
}

#endif
