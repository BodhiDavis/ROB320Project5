#pragma once

#include "rix/core/common.hpp"
#include "rix/core/impl/node_impl.hpp"
#include "rix/core/publisher.hpp"
#include "rix/core/service.hpp"
#include "rix/core/service_client.hpp"
#include "rix/core/subscriber.hpp"
#include "rix/ipc/client.hpp"
#include "rix/ipc/executor.hpp"
#include "rix/ipc/server.hpp"
#include "rix/ipc/signal_handler.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace core {

class Node {
    using Log = rix::util::Log;
    using SignalHandler = rix::ipc::SignalHandler;

   public:
    static bool init(const std::string &name, const std::string &hub_ip, uint16_t hub_port);
    static void spin(bool block);
    static void spin_once();
    static void shutdown();
    static void shutdown(const Publisher &publisher);
    static void shutdown(const Subscriber &subscriber);
    static void shutdown(const Service &service);
    static void shutdown(const ServiceClient &service_client);

    static bool ok();

    template <typename TMsg, typename TImpl = PubImplTCP>
    static Publisher advertise(const std::string &topic);

    template <typename TMsg, typename TImpl = SubImplTCP>
    static Subscriber subscribe(const std::string &topic, TSubCallback<TMsg> callback);

    template <typename TReq, typename TRes, typename TImpl = SrvImplTCP>
    static Service advertise_service(const std::string &service, TSrvCallback<TReq, TRes> callback);

    template <typename TReq, typename TRes, typename TImpl = SrvCliImplTCP>
    static ServiceClient service_client(const std::string &service);

   private:
    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;
    using MessageBase = rix::msg::MessageBase;
    using Hash = rix::msg::Hash;

    static inline uint64_t generate_id() {
        static std::random_device rd;
        static std::mt19937_64 gen(rd());
        static std::uniform_int_distribution<uint64_t> dis;
        return dis(gen);
    }

    static inline uint64_t get_machine_id() {
        std::ifstream file(rix::core::MACHINE_ID_FILE);
        uint64_t machine_id;
        if (file.is_open()) {
            file >> machine_id;
            file.close();
        } else {
            machine_id = generate_id();
            std::ofstream file(rix::core::MACHINE_ID_FILE);
            file << machine_id;
            file.close();
        }
        return machine_id;
    }

    static void sigint_handler(int signal);

    static NodeImpl impl;
};

template <typename TMsg, typename TImpl>
Publisher Node::advertise(const std::string &topic) {
    static_assert(std::is_base_of<MessageBase, TMsg>::value, "TMsg must be a subclass of Msg.");
    static_assert(std::is_base_of<PubImplBase, TImpl>::value, "TImpl must be a subclass of PubImplBase.");

    uint64_t pub_id = generate_id();
    Hash msg_hash = Serializer<TMsg>::hash(TMsg());
    std::shared_ptr<PubImplBase> pub_impl(new TImpl(pub_id, impl.get_info().id, topic, msg_hash));
    return impl.advertise(pub_impl);
}

template <typename TMsg, typename TImpl>
Subscriber Node::subscribe(const std::string &topic, TSubCallback<TMsg> callback) {
    static_assert(std::is_base_of<MessageBase, TMsg>::value, "TMsg must be a subclass of Msg.");
    static_assert(std::is_base_of<SubImplBase, TImpl>::value, "TImpl must be a subclass of PubImpl.");

    std::shared_ptr<SubCallbackBase> callback_ptr(new SubCallback<TMsg>(callback));
    uint64_t sub_id = generate_id();
    Hash msg_hash = Serializer<TMsg>::hash(TMsg());
    std::shared_ptr<SubImplBase> sub_impl(new TImpl(sub_id, impl.get_info().id, topic, msg_hash, callback_ptr));
    return impl.subscribe(sub_impl);
}

template <typename TReq, typename TRes, typename TImpl>
Service Node::advertise_service(const std::string &service, TSrvCallback<TReq, TRes> callback) {
    static_assert(std::is_base_of<MessageBase, TReq>::value, "TReq must be a subclass of Msg.");
    static_assert(std::is_base_of<MessageBase, TRes>::value, "TRes must be a subclass of Msg.");
    static_assert(std::is_base_of<SrvImplBase, TImpl>::value, "TImpl must be a subclass of SrvImplBase.");

    std::shared_ptr<SrvCallbackBase> callback_ptr(new SrvCallback<TReq, TRes>(callback));
    uint64_t srv_id = generate_id();
    Hash req_hash = Serializer<TReq>::hash(TReq());
    Hash res_hash = Serializer<TRes>::hash(TRes());
    std::shared_ptr<SrvImplBase> srv_impl(
        new TImpl(srv_id, impl.get_info().id, service, res_hash, req_hash, callback_ptr));
    return impl.advertise_service(srv_impl);
}

template <typename TReq, typename TRes, typename TImpl>
ServiceClient Node::service_client(const std::string &service) {
    static_assert(std::is_base_of<MessageBase, TReq>::value, "TReq must be a subclass of Msg.");
    static_assert(std::is_base_of<MessageBase, TRes>::value, "TRes must be a subclass of Msg.");
    static_assert(std::is_base_of<SrvCliImplBase, TImpl>::value, "TImpl must be a subclass of SrvCliImplBase.");

    uint64_t srv_cli_id = generate_id();
    Hash req_hash = Serializer<TReq>::hash(TReq());
    Hash res_hash = Serializer<TRes>::hash(TRes());
    std::shared_ptr<SrvCliImplBase> srv_cli_impl(
        new TImpl(srv_cli_id, impl.get_info().id, service, req_hash, res_hash));
    return impl.service_client(srv_cli_impl);
}

}  // namespace core
}  // namespace rix