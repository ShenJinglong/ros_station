#ifndef ROBORTS_SDK_HANDLE_H
#define ROBORTS_SDK_HANDLE_H

#include <vector>

#include "../protocol/protocol.h"
#include "dispatch.h"
#include "execution.h"

namespace roborts_sdk {

class SubscriptionBase;
class PublisherBase;
class ClientBase;
class ServiceBase;
class Executor;

template<typename Cmd>
class Subscription;
template<typename Cmd>
class Publisher;
template<typename Cmd, typename Ack>
class Client;
template<typename Cmd, typename Ack>
class Service;

class Handle: public std::enable_shared_from_this<Handle> {
public:
    template<typename Cmd>
    friend class Subscription;
    template<typename Cmd>
    friend class Publisher;
    template<typename Cmd, typename Ack>
    friend class Client;
    template<typename Cmd, typename Ack>
    friend class Service;

    explicit Handle(std::string serial_port);

    bool Init();
    std::shared_ptr<Protocol> &GetProtocol();
    
    template<typename Cmd>
    std::shared_ptr<Subscription<Cmd>> CreateSubscriber(uint8_t cmd_set, uint8_t cmd_id,
                                                        uint8_t sender, uint8_t receiver,
                                                        typename Subscription<Cmd>::CallbackType &&function) {
        auto subscriber = std::make_shared<Subscription<Cmd>>(shared_from_this(), 
                                                                cmd_set, cmd_id,
                                                                sender, receiver,
                                                                std::forward<typename Subscription<Cmd>::CallbackType>(function));
        subscription_factory_.push_back(std::dynamic_pointer_cast<SubscriptionBase>(subscriber));
        return subscriber;
    }

    template<typename Cmd>
    std::shared_ptr<Publisher<Cmd>> CreatePublisher(uint8_t cmd_set, uint8_t cmd_id, 
                                                    uint8_t sender, uint8_t receiver) {
        auto publisher = std::make_shared<Publisher<Cmd>>(shared_from_this(),
                                                            cmd_set, cmd_id,
                                                            sender, receiver);
        publisher_factory_.push_back(std::dynamic_pointer_cast<PublisherBase>(publisher));
        return publisher;
    }

    template<typename Cmd, typename Ack>
    std::shared_ptr<Service<Cmd, Ack>> CreateServer(uint8_t cmd_set, uint8_t cmd_id,
                                                    uint8_t sender, uint8_t receiver,
                                                    typename Service<Cmd, Ack>::CallbackType &&function) {
        auto service = std::make_shared<Service<Cmd, Ack>>(shared_from_this(),
                                                            cmd_set, cmd_id,
                                                            sender, receiver,
                                                            std::forward<typename Service<Cmd, Ack>::CallbackType>(function));
        service_factory_.push_back(std::dynamic_pointer_cast<ServiceBase>(service));
        return service;
    }

    template<typename Cmd, typename Ack>
    std::shared_ptr<Client<Cmd, Ack>> CreateClient(uint8_t cmd_set, uint8_t cmd_id,
                                                    uint8_t sender, uint8_t receiver) {
        auto client = std::make_shared<Client<Cmd, Ack>>(shared_from_this(),
                                                        cmd_set, cmd_id,
                                                        sender, receiver);
        client_factory_.push_back(std::dynamic_pointer_cast<ClientBase>(client));
        return client;
    }

    void Spin();

private:
    std::vector<std::shared_ptr<SubscriptionBase>> subscription_factory_;
    std::vector<std::shared_ptr<PublisherBase>> publisher_factory_;
    std::vector<std::shared_ptr<ServiceBase>> service_factory_;
    std::vector<std::shared_ptr<ClientBase>> client_factory_;

    std::shared_ptr<Executor> executor_;
    std::shared_ptr<SerialDevice> device_;
    std::shared_ptr<Protocol> protocol_;

    std::string serial_port_;
};

}

#endif