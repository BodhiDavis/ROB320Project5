#pragma once

#include "rix/core/common.hpp"
#include "rix/core/subscriber/sub_callback_base.hpp"
#include "rix/ipc/endpoint.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/PubInfo.hpp"
#include "rix/msg/mediator/SubInfo.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace core {

class SubImplBase {
    using PubInfo = rix::msg::mediator::PubInfo;
    using SubInfo = rix::msg::mediator::SubInfo;
    using Endpoint = rix::ipc::Endpoint;

   public:
    SubImplBase(uint64_t id, uint64_t node_id, const std::string &topic, const msg::Hash &msg_hash,
                std::shared_ptr<SubCallbackBase> callback, uint8_t protocol);

    SubImplBase(const SubImplBase &) = delete;
    SubImplBase &operator=(const SubImplBase &) = delete;

    ~SubImplBase();

    void shutdown();
    bool ok() const;
    const SubInfo &get_info();

    virtual void add_publishers(const std::vector<PubInfo> &pubs) = 0;
    virtual void remove_publishers(const std::vector<PubInfo> &pubs) = 0;
    virtual size_t get_num_publishers() const = 0;

   protected:
    std::atomic<bool> shutdown_flag;
    std::shared_ptr<SubCallbackBase> callback;
    SubInfo info;

    virtual Endpoint get_endpoint() const;
};

}  // namespace core
}  // namespace rix