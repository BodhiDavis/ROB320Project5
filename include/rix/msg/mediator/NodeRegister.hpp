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

class NodeRegister : public MessageBase {
  public:
    uint64_t machine_id;
    uint64_t id;
    std::string name;
    mediator::Endpoint endpoint;

    NodeRegister() = default;
    NodeRegister(const NodeRegister &other) = default;
    ~NodeRegister() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(machine_id);
        size += size_base(id);
        size += size_string(name);
        size += size_custom(endpoint);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xa249412b99cb22c3ULL, 0xbb1fbb189b27cc53ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(machine_id, buffer);
        serialize_base(id, buffer);
        serialize_string(name, buffer);
        serialize_custom(endpoint, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(machine_id, buffer, offset);
        deserialize_base(id, buffer, offset);
        deserialize_string(name, buffer, offset);
        deserialize_custom(endpoint, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix