#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Point.hpp"
#include "rix/msg/geometry/Quaternion.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Pose : public MessageBase {
  public:
    geometry::Point position;
    geometry::Quaternion orientation;

    Pose() = default;
    Pose(const Pose &other) = default;
    ~Pose() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(position);
        size += size_custom(orientation);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x9957af06de95d278ULL, 0x95bdc7c9bff3ca20ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(position, buffer);
        serialize_custom(orientation, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(position, buffer, offset);
        deserialize_custom(orientation, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix