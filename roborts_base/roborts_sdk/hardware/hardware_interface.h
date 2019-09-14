#ifndef ROBORTS_SDK_HARDWARE_INTERFACE_H
#define ROBORTS_SDK_HARDWARE_INTERFACE_H

namespace roborts_sdk {

#include <stdint.h>

class HardwareInterface {
public:
    HardwareInterface(){};
    virtual ~HardwareInterface() = default;
protected:
    virtual bool Init() = 0;
    virtual int Read(uint8_t *buf, int len) = 0;
    virtual int Write(const uint8_t *buf, int len) = 0;
};

}

#endif