#pragma once

#include "rix/core/publisher/pub_impl_base.hpp"

namespace rix {
namespace core {

class PubImplMock : public PubImplBase {
    using PubInfo = rix::msg::mediator::PubInfo;
    using SubInfo = rix::msg::mediator::SubInfo;
    using Endpoint = rix::ipc::Endpoint;
    using Log = rix::util::Log;

    int num_subscribers = 0;

   public:
    PubImplMock(uint64_t pub_id, uint64_t id, const std::string &topic, const msg::Hash &msg_hash)
        : PubImplBase(pub_id, id, topic, msg_hash, Protocol::TCP) {};

    ~PubImplMock();

    void publish(const ipc::Buffer &buffer) override { Log::debug << "PubImplMock::publish called." << std::endl; }
    void add_subscribers(const std::vector<SubInfo> &subs) override {
        Log::debug << "PubImplMock::accept_connections called." << std::endl;
        num_subscribers += subs.size();
    }
    void remove_subscribers(const std::vector<SubInfo> &subs) override {
        Log::debug << "PubImplMock::remove_connections called." << std::endl;
        num_subscribers -= subs.size();
    }
    size_t get_num_subscribers() const override {
        Log::debug << "PubImplMock::get_num_susbcribers called." << std::endl;
        return num_subscribers;
    }
};

}  // namespace core
}  // namespace rix