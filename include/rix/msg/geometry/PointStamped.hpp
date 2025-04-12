#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Point.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class PointStamped : public MessageBase {
  public:
    standard::Header header;
    geometry::Point point;

    PointStamped() = default;
    PointStamped(const PointStamped &other) = default;
    ~PointStamped() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(header);
        size += size_custom(point);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xb4c8d08a36fdcdbeULL, 0xfdf71e91db7abac5ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(header, buffer);
        serialize_custom(point, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(header, buffer, offset);
        deserialize_custom(point, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix