#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Vector3.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Twist : public MessageBase {
  public:
    geometry::Vector3 linear;
    geometry::Vector3 angular;

    Twist() = default;
    Twist(const Twist &other) = default;
    ~Twist() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(linear);
        size += size_custom(angular);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xb175febf3e100e22ULL, 0x8aae9caa149289d5ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(linear, buffer);
        serialize_custom(angular, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(linear, buffer, offset);
        deserialize_custom(angular, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix