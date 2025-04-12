#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/Endpoint.hpp"
#include "rix/msg/mediator/Hash.hpp"

namespace rix {
namespace msg {
namespace mediator {

class ActInfo : public MessageBase {
  public:
    std::string name;
    uint64_t id;
    uint64_t node_id;
    uint8_t protocol;
    mediator::Endpoint endpoint;
    mediator::Hash goal_hash;
    mediator::Hash feedback_hash;
    mediator::Hash result_hash;

    ActInfo() = default;
    ActInfo(const ActInfo &other) = default;
    ~ActInfo() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_string(name);
        size += size_base(id);
        size += size_base(node_id);
        size += size_base(protocol);
        size += size_custom(endpoint);
        size += size_custom(goal_hash);
        size += size_custom(feedback_hash);
        size += size_custom(result_hash);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x24d9a09ff0eeeaefULL, 0x2cc39a781df8d326ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_string(name, buffer);
        serialize_base(id, buffer);
        serialize_base(node_id, buffer);
        serialize_base(protocol, buffer);
        serialize_custom(endpoint, buffer);
        serialize_custom(goal_hash, buffer);
        serialize_custom(feedback_hash, buffer);
        serialize_custom(result_hash, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_string(name, buffer, offset);
        deserialize_base(id, buffer, offset);
        deserialize_base(node_id, buffer, offset);
        deserialize_base(protocol, buffer, offset);
        deserialize_custom(endpoint, buffer, offset);
        deserialize_custom(goal_hash, buffer, offset);
        deserialize_custom(feedback_hash, buffer, offset);
        deserialize_custom(result_hash, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix