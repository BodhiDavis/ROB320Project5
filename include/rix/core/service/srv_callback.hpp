#pragma once

#include "rix/core/common.hpp"
#include "rix/core/service/srv_callback_base.hpp"
#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/standard/UInt32.hpp"

namespace rix {
namespace core {

template <typename TRequest, typename TResponse>
class SrvCallback : public SrvCallbackBase {
    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;
    using UInt32 = rix::msg::standard::UInt32;
    using MessageBase = rix::msg::MessageBase;

    static_assert(std::is_base_of<MessageBase, TRequest>::value, "TRequest must be a subclass of Msg.");
    static_assert(std::is_base_of<MessageBase, TResponse>::value, "TResponse must be a subclass of Msg.");

   public:
    void call(const ipc::Buffer& request, ipc::Buffer& response) override {
        TRequest req;
        size_t offset = 0;
        Serializer<TRequest>::deserialize(req, request, offset);
        TResponse res;
        _cb(req, res);
        UInt32 msg_len;
        msg_len.data = Serializer<TResponse>::size(res);
        Serializer<UInt32>::serialize(msg_len, response);
        Serializer<TResponse>::serialize(res, response);
    }

    SrvCallback(TSrvCallback<TRequest, TResponse> cb) : _cb(cb) {}

   private:
    TSrvCallback<TRequest, TResponse> _cb;
};

}  // namespace core
}  // namespace rix
