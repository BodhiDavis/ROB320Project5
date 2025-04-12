#pragma once

#include "rix/core/common.hpp"
#include "rix/ipc/endpoint.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/SrvRequest.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace core {

class SrvCliImplBase {
    using SrvRequest = rix::msg::mediator::SrvRequest;
    using Endpoint = rix::ipc::Endpoint;

   public:
    SrvCliImplBase(uint64_t id, uint64_t node_id, const std::string &service, const msg::Hash &request_hash,
                   const msg::Hash &response_hash, uint8_t protocol);
    SrvCliImplBase(const SrvCliImplBase &) = delete;
    SrvCliImplBase &operator=(const SrvCliImplBase &) = delete;
    ~SrvCliImplBase();

    void shutdown();
    bool ok() const;
    virtual bool call(const ipc::Buffer &request, ipc::Buffer &response) = 0;
    SrvRequest get_request();
    void set_endpoint(const Endpoint &ep);

   protected:
    std::atomic<bool> shutdown_flag;
    SrvRequest request;
    Endpoint ep;
};

using SrvClientImplPtr = std::shared_ptr<SrvCliImplBase>;

}  // namespace core
}  // namespace rix