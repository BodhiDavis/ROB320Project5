#pragma once

#include "rix/core/common.hpp"
#include "rix/core/service/srv_callback_base.hpp"
#include "rix/ipc/endpoint.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/SrvInfo.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace core {

class Node;

class SrvImplBase {
    using SrvInfo = rix::msg::mediator::SrvInfo;
    using Endpoint = rix::ipc::Endpoint;

   public:
    SrvImplBase(uint64_t id, uint64_t node_id, const std::string &service, const msg::Hash &request_hash,
                const msg::Hash &response_hash, std::shared_ptr<SrvCallbackBase> callback, uint8_t protocol);
    SrvImplBase(const SrvImplBase &) = delete;
    SrvImplBase &operator=(const SrvImplBase &) = delete;
    ~SrvImplBase();

    void shutdown();
    bool ok() const;
    const SrvInfo &get_info();

   protected:
    std::atomic<bool> shutdown_flag;
    std::shared_ptr<SrvCallbackBase> callback;
    SrvInfo info;

    virtual Endpoint get_endpoint() const = 0;
};

using SrvImplPtr = std::shared_ptr<SrvImplBase>;

}  // namespace core
}  // namespace rix