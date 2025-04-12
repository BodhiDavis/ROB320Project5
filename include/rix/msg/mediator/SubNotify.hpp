#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/PubInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SubNotify : public MessageBase {
  public:
    uint64_t id;
    bool connect;
    uint8_t error;
    std::vector<mediator::PubInfo> publishers;

    SubNotify() = default;
    SubNotify(const SubNotify &other) = default;
    ~SubNotify() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(id);
        size += size_base(connect);
        size += size_base(error);
        size += size_custom_vec(publishers);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x2a29d5c5507d0d20ULL, 0x821148e57874b0f3ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(id, buffer);
        serialize_base(connect, buffer);
        serialize_base(error, buffer);
        serialize_custom_vec(publishers, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(id, buffer, offset);
        deserialize_base(connect, buffer, offset);
        deserialize_base(error, buffer, offset);
        deserialize_custom_vec(publishers, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix