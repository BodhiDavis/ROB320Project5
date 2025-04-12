#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace msg {
namespace mediator {

class Status : public MessageBase {
  public:
    uint64_t id;
    uint8_t error;

    Status() = default;
    Status(const Status &other) = default;
    ~Status() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(id);
        size += size_base(error);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xddff3efefe758f97ULL, 0x974f8aae66a5d9f6ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(id, buffer);
        serialize_base(error, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(id, buffer, offset);
        deserialize_base(error, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix