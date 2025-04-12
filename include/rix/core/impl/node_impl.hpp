#pragma once

#include "rix/core/common.hpp"
#include "rix/core/publisher.hpp"
#include "rix/core/service.hpp"
#include "rix/core/service_client.hpp"
#include "rix/core/subscriber.hpp"
#include "rix/ipc/client.hpp"
#include "rix/ipc/server.hpp"
#include "rix/ipc/signal_handler.hpp"
#include "rix/msg/mediator/ActInfo.hpp"
#include "rix/msg/mediator/ActRequest.hpp"
#include "rix/msg/mediator/ActResponse.hpp"
#include "rix/msg/mediator/Endpoint.hpp"
#include "rix/msg/mediator/Hash.hpp"
#include "rix/msg/mediator/NodeInfo.hpp"
#include "rix/msg/mediator/Operation.hpp"
#include "rix/msg/mediator/PubInfo.hpp"
#include "rix/msg/mediator/PubNotify.hpp"
#include "rix/msg/mediator/SrvInfo.hpp"
#include "rix/msg/mediator/SrvRequest.hpp"
#include "rix/msg/mediator/SrvResponse.hpp"
#include "rix/msg/mediator/Status.hpp"
#include "rix/msg/mediator/SubInfo.hpp"
#include "rix/msg/mediator/SubNotify.hpp"
#include "rix/msg/mediator/TopicInfo.hpp"
#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/util/log.hpp"

namespace rix {
namespace core {

class NodeImpl {
   private:
    using Log = rix::util::Log;
    using Buffer = rix::ipc::Buffer;
    template <typename TMsg>
    using Serializer = rix::msg::Serializer<TMsg>;

    using Operation = rix::msg::mediator::Operation;

    using NodeInfo = rix::msg::mediator::NodeInfo;
    using SubInfo = rix::msg::mediator::SubInfo;
    using PubInfo = rix::msg::mediator::PubInfo;
    using SrvInfo = rix::msg::mediator::SrvInfo;
    using ActInfo = rix::msg::mediator::ActInfo;
    using TopicInfo = rix::msg::mediator::TopicInfo;
    using Endpoint = rix::msg::mediator::Endpoint;
    using Hash = rix::msg::mediator::Hash;

    using Status = rix::msg::mediator::Status;
    using SubNotify = rix::msg::mediator::SubNotify;
    using PubNotify = rix::msg::mediator::PubNotify;

    using SrvRequest = rix::msg::mediator::SrvRequest;
    using ActRequest = rix::msg::mediator::ActRequest;
    using SrvResponse = rix::msg::mediator::SrvResponse;
    using ActResponse = rix::msg::mediator::ActResponse;

   public:
    NodeImpl();
    ~NodeImpl();

    bool init(uint64_t id, uint64_t machine_id, const std::string &name, const std::string &hub_ip, uint16_t hub_port);
    void spin(bool block);
    void spin_once();
    void shutdown();
    void shutdown(const Publisher &publisher);
    void shutdown(const Subscriber &subscriber);
    void shutdown(const Service &service);
    void shutdown(const ServiceClient &service_client);
    bool ok();
    const NodeInfo &get_info();
    Publisher advertise(std::shared_ptr<PubImplBase> impl);
    Subscriber subscribe(std::shared_ptr<SubImplBase> impl);
    Service advertise_service(std::shared_ptr<SrvImplBase> impl);
    ServiceClient service_client(std::shared_ptr<SrvCliImplBase> impl);

   private:
    void on_accept_hub_conn(int status);
    bool send_node_register();
    bool send_pub_register(const PubInfo &pub_info);
    bool send_sub_register(const SubInfo &sub_info);
    bool send_srv_register(const SrvInfo &srv_info);
    bool send_srv_request(const SrvRequest &request, SrvResponse &response);

    bool send_node_deregister();
    bool send_sub_deregister(const SubInfo &sub_info);
    bool send_pub_deregister(const PubInfo &pub_info);
    bool send_srv_deregister(const SrvInfo &srv_info);

    void handle_pub_notify(const PubNotify &pub_notify);
    void handle_sub_notify(const SubNotify &sub_notify);

    bool hub_rpc(const Buffer &send_buffer, Buffer &recv_buffer, uint8_t &opcode);
    bool hub_rpc(const Buffer &send_buffer, Buffer &recv_buffer);
    bool hub_rpc(const Buffer &send_buffer);

    NodeInfo info;
    ipc::Endpoint hub_ep;
    ipc::Server server;
    std::weak_ptr<ipc::Connection> hub_conn;
    Buffer hub_buffer;
    std::mutex mutex;
    std::thread spin_thread;

    std::atomic<bool> spinning;
    std::atomic<bool> shutdown_flag;

    std::unordered_map<uint64_t, Publisher> publishers;
    std::unordered_map<uint64_t, Subscriber> subscribers;
    std::unordered_map<uint64_t, Service> services;
    std::unordered_map<uint64_t, ServiceClient> service_clients;
};

}  // namespace core
}  // namespace rix