#pragma once

#include <map>

#include "rix/core/common.hpp"
#include "rix/core/service_client/srv_cli_impl_base.hpp"
#include "rix/msg/mediator/SrvInfo.hpp"
#include "rix/msg/mediator/SrvRequest.hpp"
#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/standard/UInt32.hpp"

namespace rix {
namespace core {

class NodeImpl;

class ServiceClient {
    friend class NodeImpl;

    using MessageBase = rix::msg::MessageBase;
    using SrvRequest = rix::msg::mediator::SrvRequest;
    using UInt32 = rix::msg::standard::UInt32;

    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

   public:
    ServiceClient();
    ServiceClient(const ServiceClient &other);
    ~ServiceClient();
    ServiceClient &operator=(const ServiceClient &other);

    std::string get_service() const;
    bool ok() const;

    template <typename TRequest, typename TResponse>
    bool call(const TRequest &request, TResponse &response) {
        static_assert(std::is_base_of<MessageBase, TRequest>::value, "TRequest must be a subclass of Msg.");
        static_assert(std::is_base_of<MessageBase, TResponse>::value, "TResponse must be a subclass of Msg.");

        if (!ok()) {
            return false;
        }

        std::lock_guard<std::mutex> guard(impl_mtx);
        msg::Hash request_hash = Serializer<TRequest>::hash(request);
        msg::Hash response_hash = Serializer<TResponse>::hash(response);
        const SrvRequest &srv_req = impl->get_request();
        if (request_hash != srv_req.request_hash.value || response_hash != srv_req.response_hash.value) {
            return false;
        }
        ipc::Buffer request_buffer, response_buffer;
        UInt32 size;
        size.data = Serializer<TRequest>::size(request);
        Serializer<UInt32>::serialize(size, request_buffer);
        Serializer<TRequest>::serialize(request, request_buffer);
        bool status = impl->call(request_buffer, response_buffer);
        if (!status) {
            return false;
        }
        size_t offset = 0;
        Serializer<TResponse>::deserialize(response, response_buffer, offset);
        return true;
    }

   private:
    mutable std::mutex impl_mtx;
    std::shared_ptr<SrvCliImplBase> impl;

    ServiceClient(std::shared_ptr<SrvCliImplBase> impl);
};

}  // namespace core
}  // namespace rix