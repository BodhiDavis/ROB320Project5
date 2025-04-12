#pragma once

#include "rix/core/common.hpp"
#include "rix/core/publisher/pub_impl_base.hpp"
#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/standard/UInt32.hpp"

namespace rix {
namespace core {

class NodeImpl;

class Publisher {
    friend class NodeImpl;

    using MessageBase = rix::msg::MessageBase;
    using PubInfo = rix::msg::mediator::PubInfo;
    using SubInfo = rix::msg::mediator::SubInfo;
    using UInt32 = rix::msg::standard::UInt32;

    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

   public:
    Publisher();
    ~Publisher();
    Publisher(const Publisher &other);
    Publisher &operator=(const Publisher &other);

    std::string get_topic() const;
    size_t get_num_subscribers() const;
    bool ok() const;

    template <typename TMsg>
    void publish(const TMsg &msg) {
        static_assert(std::is_base_of<MessageBase, TMsg>::value, "TMsg must be a subclass of Msg.");
        if (!ok()) {
            return;
        }
        std::lock_guard<std::mutex> guard(impl_mtx);
        msg::Hash msg_hash = Serializer<TMsg>::hash(msg);
        if (msg_hash != impl->get_info().topic_info.message_hash.value) {
            return;
        }
        ipc::Buffer buffer;
        UInt32 size;
        size.data = Serializer<TMsg>::size(msg);
        Serializer<UInt32>::serialize(size, buffer);
        Serializer<TMsg>::serialize(msg, buffer);
        impl->publish(buffer);
    }

   private:
    std::shared_ptr<PubImplBase> impl;
    mutable std::mutex impl_mtx;

    Publisher(std::shared_ptr<PubImplBase> impl);
    void shutdown();
    void add_subscribers(const std::vector<SubInfo> &subs);
    void remove_subscribers(const std::vector<SubInfo> &subs);
};

}  // namespace core
}  // namespace rix