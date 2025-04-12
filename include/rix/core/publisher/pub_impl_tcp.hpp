#pragma once

#include "rix/core/publisher/pub_impl_base.hpp"
#include "rix/ipc/server.hpp"

namespace rix {
namespace core {

class PubImplTCP : public PubImplBase {
    using Log = rix::util::Log;
    using Operation = rix::msg::mediator::Operation;
    using PubInfo = rix::msg::mediator::PubInfo;
    using SubInfo = rix::msg::mediator::SubInfo;
    using Endpoint = rix::ipc::Endpoint;

    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

    ipc::Server server;
    size_t to_accept = 0;
    std::weak_ptr<ipc::Connection> sub_conn;
    std::unordered_map<uint64_t, std::weak_ptr<ipc::Connection>> subscribers;

   public:
    PubImplTCP(uint64_t pub_id, uint64_t id, const std::string &topic, const msg::Hash &msg_hash);
    ~PubImplTCP();

    void publish(const ipc::Buffer &buffer) override;
    void add_subscribers(const std::vector<SubInfo> &subs) override;
    void remove_subscribers(const std::vector<SubInfo> &subs) override;
    size_t get_num_subscribers() const override;
    Endpoint get_endpoint() const override;

   private:
    void on_accept(int status);
};

}  // namespace core
}  // namespace rix