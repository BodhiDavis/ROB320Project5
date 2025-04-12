#pragma once

#include "rix/core/common.hpp"
#include "rix/core/subscriber/sub_impl_base.hpp"
#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace core {

class NodeImpl;

class Subscriber {
    friend class NodeImpl;

    using MessageBase = rix::msg::MessageBase;
    using PubInfo = rix::msg::mediator::PubInfo;
    using SubInfo = rix::msg::mediator::SubInfo;

   public:
    Subscriber();
    Subscriber(const Subscriber &other);
    ~Subscriber();
    Subscriber &operator=(const Subscriber &other);
    
    std::string get_topic() const;
    int get_num_publishers() const;
    bool ok() const;

   private:
    std::shared_ptr<SubImplBase> impl;
    mutable std::mutex impl_mtx;

    Subscriber(std::shared_ptr<SubImplBase> impl);
    void shutdown();
    void add_publishers(const std::vector<PubInfo> &pubs);
    void remove_publishers(const std::vector<PubInfo> &pubs);
};

}  // namespace core
}  // namespace rix