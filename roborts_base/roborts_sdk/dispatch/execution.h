#ifndef ROBORTS_SDK_EXECUTION_H
#define ROBORTS_SDK_EXECUTION_H

#include "dispatch.h"

namespace roborts_sdk {

class Handle;
class SubscriptionBase;
class PublisherBase;
class ClientBase;
class ServiceBase;

class Executor {
public:
    Executor(std::shared_ptr<Handle> handle);
    ~Executor() = default;

    std::shared_ptr<Handle> GetHandle();
    void ExecuteSubscription(const std::shared_ptr<SubscriptionBase> &subscription);
    void ExecuteService(const std::shared_ptr<ServiceBase> &service);
    void ExecuteClient(const std::shared_ptr<ClientBase> &client);
private:
    std::shared_ptr<Handle> handle_;
};

}

#endif