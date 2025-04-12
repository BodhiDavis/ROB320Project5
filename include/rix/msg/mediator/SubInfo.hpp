#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/Endpoint.hpp"
#include "rix/msg/mediator/TopicInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SubInfo : public MessageBase {
  public:
    uint64_t id;
    uint64_t node_id;
    uint8_t protocol;
    mediator::TopicInfo topic_info;
    mediator::Endpoint endpoint;

    SubInfo() = default;
    SubInfo(const SubInfo &other) = default;
    ~SubInfo() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(id);
        size += size_base(node_id);
        size += size_base(protocol);
        size += size_custom(topic_info);
        size += size_custom(endpoint);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x412bd445af80ed2bULL, 0x74d606be1fc5c66cULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(id, buffer);
        serialize_base(node_id, buffer);
        serialize_base(protocol, buffer);
        serialize_custom(topic_info, buffer);
        serialize_custom(endpoint, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(id, buffer, offset);
        deserialize_base(node_id, buffer, offset);
        deserialize_base(protocol, buffer, offset);
        deserialize_custom(topic_info, buffer, offset);
        deserialize_custom(endpoint, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix