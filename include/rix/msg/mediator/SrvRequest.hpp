#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/Hash.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SrvRequest : public MessageBase {
  public:
    uint64_t node_id;
    uint64_t id;
    std::string name;
    uint8_t protocol;
    mediator::Hash request_hash;
    mediator::Hash response_hash;

    SrvRequest() = default;
    SrvRequest(const SrvRequest &other) = default;
    ~SrvRequest() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(node_id);
        size += size_base(id);
        size += size_string(name);
        size += size_base(protocol);
        size += size_custom(request_hash);
        size += size_custom(response_hash);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x1384bf96a2d41d6aULL, 0x6c28b06c6b2e2f3cULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(node_id, buffer);
        serialize_base(id, buffer);
        serialize_string(name, buffer);
        serialize_base(protocol, buffer);
        serialize_custom(request_hash, buffer);
        serialize_custom(response_hash, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(node_id, buffer, offset);
        deserialize_base(id, buffer, offset);
        deserialize_string(name, buffer, offset);
        deserialize_base(protocol, buffer, offset);
        deserialize_custom(request_hash, buffer, offset);
        deserialize_custom(response_hash, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix