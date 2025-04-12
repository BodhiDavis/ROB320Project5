#pragma once

#include "rix/core/common.hpp"
#include "rix/core/subscriber/sub_callback_base.hpp"
#include "rix/core/subscriber/sub_impl_base.hpp"
#include "rix/ipc/client.hpp"
#include "rix/msg/standard/UInt32.hpp"

namespace rix {
namespace core {

class SubImplTCP : public SubImplBase {
    using Log = rix::util::Log;
    using PubInfo = rix::msg::mediator::PubInfo;
    using Operation = rix::msg::mediator::Operation;
    using SubInfo = rix::msg::mediator::SubInfo;
    using UInt32 = rix::msg::standard::UInt32;
    using Endpoint = rix::ipc::Endpoint;

    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

    std::map<uint64_t, ipc::Client> clients;
    std::mutex buffer_mtx;
    ipc::Buffer buffer;

   public:
    SubImplTCP(uint64_t id, uint64_t node_id, const std::string &topic, const msg::Hash &msg_hash,
               std::shared_ptr<SubCallbackBase> callback);
    ~SubImplTCP();

    void on_recv(ipc::Client &client, int status, size_t size);
    virtual void add_publishers(const std::vector<PubInfo> &pubs) override;
    virtual void remove_publishers(const std::vector<PubInfo> &pubs) override;
    virtual size_t get_num_publishers() const override;
};

}  // namespace core
}  // namespace rix