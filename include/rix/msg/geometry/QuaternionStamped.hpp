#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Quaternion.hpp"
#include "rix/msg/standard/Header.hpp"

namespace rix {
namespace msg {
namespace geometry {

class QuaternionStamped : public MessageBase {
  public:
    standard::Header header;
    geometry::Quaternion quaternion;

    QuaternionStamped() = default;
    QuaternionStamped(const QuaternionStamped &other) = default;
    ~QuaternionStamped() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(header);
        size += size_custom(quaternion);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x7903e2b08f540c91ULL, 0x9f7884a88703a9d5ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(header, buffer);
        serialize_custom(quaternion, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(header, buffer, offset);
        deserialize_custom(quaternion, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix