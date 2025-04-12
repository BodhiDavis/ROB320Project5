#pragma once

#include "rix/core/common.hpp"
#include "rix/core/service/srv_callback_base.hpp"
#include "rix/core/service/srv_impl_base.hpp"

namespace rix {
namespace core {

class NodeImpl;

class Service {
    friend class NodeImpl;

    using MessageBase = rix::msg::MessageBase;
    using SrvInfo = rix::msg::mediator::SrvInfo;

   public:
    Service();
    Service(const Service &other);
    ~Service();
    Service &operator=(const Service &other);

    std::string get_service() const;
    bool ok() const;

   protected:
    std::shared_ptr<SrvImplBase> impl;
    mutable std::mutex impl_mtx;

    Service(std::shared_ptr<SrvImplBase> impl);
    void shutdown();
};

}  // namespace core
}  // namespace rix