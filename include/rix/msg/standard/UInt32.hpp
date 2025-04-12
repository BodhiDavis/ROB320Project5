#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace msg {
namespace standard {

class UInt32 : public MessageBase {
  public:
    uint32_t data;

    UInt32() = default;
    UInt32(const UInt32 &other) = default;
    ~UInt32() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(data);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xd8f4d2578b356851ULL, 0x0cca4de80ef81716ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(data, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(data, buffer, offset);
    }
};

} // namespace standard
} // namespace msg
} // namespace rix