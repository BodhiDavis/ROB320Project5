#pragma once

#include "rix/core/common.hpp"
#include "rix/core/service/srv_callback_base.hpp"
#include "rix/core/service/srv_impl_base.hpp"
#include "rix/core/subscriber/sub_callback_base.hpp"
#include "rix/core/subscriber/sub_impl_base.hpp"
#include "rix/ipc/server.hpp"
#include "rix/msg/standard/UInt32.hpp"

namespace rix {
namespace core {

class SrvImplTCP : public SrvImplBase {
    using Log = rix::util::Log;
    using UInt32 = rix::msg::standard::UInt32;

    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

    ipc::Server server;
    std::weak_ptr<ipc::Connection> conn;

   public:
    SrvImplTCP(uint64_t id, uint64_t node_id, const std::string &service, const msg::Hash &request_hash,
               const msg::Hash &response_hash, std::shared_ptr<SrvCallbackBase> callback);
    ~SrvImplTCP();

    virtual ipc::Endpoint get_endpoint() const override;

   private:
    void on_accept(int status);
};

}  // namespace core
}  // namespace rix