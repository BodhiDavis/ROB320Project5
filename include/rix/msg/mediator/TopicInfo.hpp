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

class TopicInfo : public MessageBase {
  public:
    std::string name;
    mediator::Hash message_hash;

    TopicInfo() = default;
    TopicInfo(const TopicInfo &other) = default;
    ~TopicInfo() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_string(name);
        size += size_custom(message_hash);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x9b519e75be48df0fULL, 0xa9835ec2a3bc53f2ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_string(name, buffer);
        serialize_custom(message_hash, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_string(name, buffer, offset);
        deserialize_custom(message_hash, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix