#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Vector3.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Vector3Stamped : public MessageBase {
  public:
    standard::Header header;
    geometry::Vector3 vector3;

    Vector3Stamped() = default;
    Vector3Stamped(const Vector3Stamped &other) = default;
    ~Vector3Stamped() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(header);
        size += size_custom(vector3);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xf1c636a9f5a7f966ULL, 0xa6b484cbddf98d98ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(header, buffer);
        serialize_custom(vector3, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(header, buffer, offset);
        deserialize_custom(vector3, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix