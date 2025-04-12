#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/SubInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class PubNotify : public MessageBase {
  public:
    uint64_t id;
    bool connect;
    uint8_t error;
    std::vector<mediator::SubInfo> subscribers;

    PubNotify() = default;
    PubNotify(const PubNotify &other) = default;
    ~PubNotify() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(id);
        size += size_base(connect);
        size += size_base(error);
        size += size_custom_vec(subscribers);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x35b3fa6a2ec8563eULL, 0x39c1ce93fb200921ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(id, buffer);
        serialize_base(connect, buffer);
        serialize_base(error, buffer);
        serialize_custom_vec(subscribers, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(id, buffer, offset);
        deserialize_base(connect, buffer, offset);
        deserialize_base(error, buffer, offset);
        deserialize_custom_vec(subscribers, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix