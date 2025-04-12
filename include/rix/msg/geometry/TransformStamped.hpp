#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class TransformStamped : public MessageBase {
  public:
    standard::Header header;
    std::string child_frame_id;
    geometry::Transform transform;

    TransformStamped() = default;
    TransformStamped(const TransformStamped &other) = default;
    ~TransformStamped() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(header);
        size += size_string(child_frame_id);
        size += size_custom(transform);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x947e539bf458e0d2ULL, 0xb4adc369bac4840eULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(header, buffer);
        serialize_string(child_frame_id, buffer);
        serialize_custom(transform, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(header, buffer, offset);
        deserialize_string(child_frame_id, buffer, offset);
        deserialize_custom(transform, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix