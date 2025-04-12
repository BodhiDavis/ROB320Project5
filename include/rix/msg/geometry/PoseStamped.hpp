#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Pose.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class PoseStamped : public MessageBase {
  public:
    standard::Header header;
    geometry::Pose pose;

    PoseStamped() = default;
    PoseStamped(const PoseStamped &other) = default;
    ~PoseStamped() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(header);
        size += size_custom(pose);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xc42cedea08f426f8ULL, 0x1360043a6733dbc8ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(header, buffer);
        serialize_custom(pose, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(header, buffer, offset);
        deserialize_custom(pose, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix