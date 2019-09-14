#ifndef ROBORTS_BASE_CONFIG_H
#define ROBORTS_BASE_CONFIG_H

#include <ros/ros.h>

namespace roborts_base {

struct Config {
    void GetParam(ros::NodeHandle *nh) {
        nh->param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    }
    std::string serial_port;
};

}


#endif