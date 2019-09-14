#include "handle.h"

namespace roborts_sdk {

Handle::Handle(std::string serial_port) {
    serial_port_ = serial_port;
    device_ = std::make_shared<SerialDevice>(serial_port_, 921600);
    protocol_ = std::make_shared<Protocol>(device_);
}

bool Handle::Init() {
    if (!device_->Init()) {
        LOG_ERROR << "Can not open device: " << serial_port_
        << ". Please check if the USB device is inserted and connection is configured correctly!";
        return false;
    }
    LOG_INFO << "Connection to " << serial_port_;
    if (!protocol_->Init()) {
        LOG_ERROR << "Protocol initialization failed.";
        return false;
    }
    executor_ = std::make_shared<Executor>(shared_from_this());
    LOG_INFO << "Initialization of protocol layer and dispatch layer succeeded.";
    return true;
}

std::shared_ptr<Protocol> &Handle::GetProtocol() {
    return protocol_;
}

void Handle::Spin() {
    for (auto sub : subscription_factory_) {
        executor_->ExecuteSubscription(sub);
    }
    for (auto client : client_factory_) {
        executor_->ExecuteClient(client);
    }
    for (auto service : service_factory_) {
        executor_->ExecuteService(service);
    }
}

}