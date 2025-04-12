#pragma once

#include "rix/core/common.hpp"
#include "rix/core/service_client/srv_cli_impl_base.hpp"
#include "rix/msg/standard/UInt32.hpp"
#include "rix/ipc/client.hpp"

namespace rix {
namespace core {

class SrvCliImplTCP : public SrvCliImplBase {
    using UInt32 = rix::msg::standard::UInt32;

    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

   public:
    SrvCliImplTCP(uint64_t id, uint64_t node_id, const std::string &service, const msg::Hash &request_hash,
                  const msg::Hash &response_hash);
    ~SrvCliImplTCP();

    virtual bool call(const ipc::Buffer &request, ipc::Buffer &response) override;
};

}  // namespace core
}  // namespace rix