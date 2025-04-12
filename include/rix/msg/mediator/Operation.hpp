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

class Operation : public MessageBase {
  public:
    uint32_t len;
    uint8_t opcode;

    Operation() = default;
    Operation(const Operation &other) = default;
    ~Operation() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(len);
        size += size_base(opcode);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xb4e1f3ef193d591cULL, 0x333f693059e06949ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(len, buffer);
        serialize_base(opcode, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(len, buffer, offset);
        deserialize_base(opcode, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix