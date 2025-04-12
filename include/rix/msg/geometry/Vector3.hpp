#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Vector3 : public MessageBase {
  public:
    float x;
    float y;
    float z;

    Vector3() = default;
    Vector3(const Vector3 &other) = default;
    ~Vector3() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(x);
        size += size_base(y);
        size += size_base(z);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x8d6ceb30de1ffbd5ULL, 0xf86f8ed2ccb4960dULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(x, buffer);
        serialize_base(y, buffer);
        serialize_base(z, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(x, buffer, offset);
        deserialize_base(y, buffer, offset);
        deserialize_base(z, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix