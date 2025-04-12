#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/Endpoint.hpp"

namespace rix {
namespace msg {
namespace mediator {

class NodeInfo : public MessageBase {
  public:
    std::string name;
    uint64_t id;
    uint64_t machine_id;
    uint8_t protocol;
    mediator::Endpoint endpoint;

    NodeInfo() = default;
    NodeInfo(const NodeInfo &other) = default;
    ~NodeInfo() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_string(name);
        size += size_base(id);
        size += size_base(machine_id);
        size += size_base(protocol);
        size += size_custom(endpoint);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x18ab53595ad421e9ULL, 0xb7e39f4e80f68b36ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_string(name, buffer);
        serialize_base(id, buffer);
        serialize_base(machine_id, buffer);
        serialize_base(protocol, buffer);
        serialize_custom(endpoint, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_string(name, buffer, offset);
        deserialize_base(id, buffer, offset);
        deserialize_base(machine_id, buffer, offset);
        deserialize_base(protocol, buffer, offset);
        deserialize_custom(endpoint, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix