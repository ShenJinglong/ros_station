#include "execution.h"

namespace roborts_sdk {

Executor::Executor(std::shared_ptr<Handle> handle) : handle_(handle) {

}

std::shared_ptr<Handle> Executor::GetHandle() {
    return handle_;
}

void Executor::ExecuteSubscription(const std::shared_ptr<SubscriptionBase> &subscription) {
    auto message_header = subscription->CreateMessageHeader();
    std::shared_ptr<void> message = subscription->CreateMessage();

    bool ret = GetHandle()->GetProtocol()->Take(subscription->GetCommandInfo().get(), message_header.get(), message.get());
    if (ret) {
        subscription->HandleMessage(message_header, message);
    } else {

    }
}

void Executor::ExecuteService(const std::shared_ptr<ServiceBase> &service) {
    auto request_header = service->CreateRequestHeader();
    std::shared_ptr<void> request = service->CreateRequest();

    bool ret = GetHandle()->GetProtocol()->Take(service->GetCommandInfo().get(), request_header.get(), request.get());
    if (ret) {
        service->HandleRequest(request_header, request);
    } else {
        DLOG_ERROR << "take request failed!";
    }
}

void Executor::ExecuteClient(const std::shared_ptr<ClientBase> &client) {
    auto request_header = client->CreateRequestHeader();
    std::shared_ptr<void> response = client->CreateResponse();

    bool ret = GetHandle()->GetProtocol()->Take(client->GetCommandInfo().get(), request_header.get(), response.get());
    if (ret) {
        client->HandleResponse(request_header, response);
    } else {
        
    }
}

}