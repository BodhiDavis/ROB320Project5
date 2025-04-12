#pragma once

#include "rix/core/common.hpp"
#include "rix/ipc/endpoint.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/PubInfo.hpp"
#include "rix/msg/mediator/SubInfo.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace core {

class PubImplBase {
    using PubInfo = rix::msg::mediator::PubInfo;
    using SubInfo = rix::msg::mediator::SubInfo;
    using Endpoint = rix::ipc::Endpoint;

   public:
    PubImplBase(uint64_t id, uint64_t node_id, const std::string &topic, const msg::Hash &msg_hash, uint8_t protocol);
    PubImplBase(const PubImplBase &) = delete;
    PubImplBase &operator=(const PubImplBase &) = delete;
    ~PubImplBase();

    void shutdown();
    bool ok() const;
    const PubInfo &get_info();

    virtual void publish(const ipc::Buffer &buffer) = 0;
    virtual void add_subscribers(const std::vector<SubInfo> &subs) = 0;
    virtual void remove_subscribers(const std::vector<SubInfo> &subs) = 0;
    virtual size_t get_num_subscribers() const = 0;

   protected:
    std::atomic<bool> shutdown_flag;
    PubInfo info;

    virtual Endpoint get_endpoint() const;
};

}  // namespace core
}  // namespace rix