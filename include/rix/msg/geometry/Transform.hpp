#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Quaternion.hpp"
#include "rix/msg/geometry/Vector3.hpp"

namespace rix {
namespace msg {
namespace geometry {

class Transform : public MessageBase {
  public:
    geometry::Vector3 translation;
    geometry::Quaternion rotation;

    Transform() = default;
    Transform(const Transform &other) = default;
    ~Transform() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(translation);
        size += size_custom(rotation);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x5b5b8afca927e22aULL, 0x5ee43f5391ef1e4dULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(translation, buffer);
        serialize_custom(rotation, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(translation, buffer, offset);
        deserialize_custom(rotation, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix