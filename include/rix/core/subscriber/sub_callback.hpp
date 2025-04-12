#pragma once

#include "rix/core/common.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/message_base.hpp"
#include "rix/core/subscriber/sub_callback_base.hpp"

namespace rix {
namespace core {

template <typename TMsg>
class SubCallback : public SubCallbackBase {
    using Serializer = rix::msg::Serializer<TMsg>;
    using MessageBase = rix::msg::MessageBase;
    static_assert(std::is_base_of<MessageBase, TMsg>::value, "TMsg must be a subclass of Msg.");

   public:
    SubCallback(TSubCallback<TMsg> cb) : _cb(cb) {}

    void call(const ipc::Buffer &buffer) override {
        TMsg msg;
        size_t offset = 0;
        Serializer::deserialize(msg, buffer, offset);
        _cb(msg);
    }

   private:
    TSubCallback<TMsg> _cb;
};

}  // namespace core
}  // namespace rix