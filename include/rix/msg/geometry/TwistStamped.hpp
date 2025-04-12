#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Twist.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class TwistStamped : public MessageBase {
  public:
    standard::Header header;
    geometry::Twist twist;

    TwistStamped() = default;
    TwistStamped(const TwistStamped &other) = default;
    ~TwistStamped() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(header);
        size += size_custom(twist);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x03e488527e8a4c16ULL, 0x88ae78d5353ce6b8ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(header, buffer);
        serialize_custom(twist, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(header, buffer, offset);
        deserialize_custom(twist, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix