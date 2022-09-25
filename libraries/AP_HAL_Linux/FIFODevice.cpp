#include "FIFODevice.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>

FIFODevice::FIFODevice(const char *rd_device, const char *wr_device):
    _rd_device(rd_device), _wr_device(wr_device),
    _rd_fd(-1), _wr_fd(-1)
{
}

FIFODevice::~FIFODevice()
{
}

bool FIFODevice::close()
{
    bool ret = true;

    if (_rd_fd >= 0) {
        if (::close(_rd_fd) < 0) {
            ret = false;
        }
        _rd_fd = -1;
    }

    if (_wr_fd >= 0) {
        if (::close(_wr_fd) < 0) {
            ret = false;
        }
        _wr_fd = -1;
    }

    return ret;
}

bool FIFODevice::open()
{
    //Use O_RDWR behaviour to defer blocking during open.
    _rd_fd = ::open(_rd_device, O_RDWR | O_CLOEXEC | O_NOCTTY);

    if (_rd_fd < 0) {
        ::fprintf(stderr, "Failed to open RX FIFO device %s - %s\n",
                  _rd_device, strerror(errno));
        return false;
    }

    _wr_fd = ::open(_wr_device, O_RDWR | O_CLOEXEC | O_NOCTTY);

    if (_wr_fd < 0) {
        ::fprintf(stderr, "Failed to open TX FIFO device %s - %s\n",
                  _wr_device, strerror(errno));
        ::close(_rd_fd);
        _rd_fd = -1;
        return false;
    }

    return true;
}

ssize_t FIFODevice::read(uint8_t *buf, uint16_t n)
{
    return ::read(_rd_fd, buf, n);
}

ssize_t FIFODevice::write(const uint8_t *buf, uint16_t n)
{
    return ::write(_wr_fd, buf, n);
}

void FIFODevice::set_blocking(bool blocking)
{
    int rd_flags  = fcntl(_rd_fd, F_GETFL, 0);
    int wr_flags  = fcntl(_wr_fd, F_GETFL, 0);

    if (blocking) {
        rd_flags = rd_flags & ~O_NONBLOCK;
        wr_flags = wr_flags & ~O_NONBLOCK;
    } else {
        rd_flags = rd_flags | O_NONBLOCK;
        wr_flags = wr_flags | O_NONBLOCK;
    }

    if (fcntl(_rd_fd, F_SETFL, rd_flags) < 0) {
        ::fprintf(stderr, "Failed to set FIFO %s nonblocking %s\n", _rd_device, strerror(errno));
    }

    if (fcntl(_wr_fd, F_SETFL, wr_flags) < 0) {
        ::fprintf(stderr, "Failed to set FIFO %s nonblocking %s\n", _wr_device, strerror(errno));
    }
}

void FIFODevice::set_speed(uint32_t baudrate)
{
}

void FIFODevice::set_flow_control(AP_HAL::UARTDriver::flow_control flow_control_setting)
{
}

void FIFODevice::set_parity(int v)
{
}
