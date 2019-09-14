#ifndef ROBORTS_SDK_SERIAL_DEVICE_H
#define ROBORTS_SDK_SERIAL_DEVICE_H

#include <string>
#include <cstring>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "../utilities/log.h"
#include "hardware_interface.h"

namespace roborts_sdk {

class SerialDevice: public HardwareInterface {
public:
    SerialDevice(std::string port_name, int baudrate);
    ~SerialDevice();
    virtual bool Init() override;
    virtual int Read(uint8_t *buf, int len) override;
    virtual int Write(const uint8_t *buf, int len) override;
private:
    bool OpenDevice();
    bool CloseDevice();
    bool ConfigDevice();

    std::string port_name_;
    int baudrate_;
    int stop_bits_;
    int data_bits_;
    char parity_bits_;
    int serial_fd_;
    fd_set serial_fd_set_;
    struct termios new_termios_, old_termios_;
};

}

#endif