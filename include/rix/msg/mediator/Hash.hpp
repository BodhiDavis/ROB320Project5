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

class Hash : public MessageBase {
  public:
    std::array<uint64_t, 2> value;

    Hash() = default;
    Hash(const Hash &other) = default;
    ~Hash() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base_arr(value);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xd3c651d633dc7df8ULL, 0xe0b3d8b8a9e0d5c0ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base_arr(value, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base_arr(value, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix