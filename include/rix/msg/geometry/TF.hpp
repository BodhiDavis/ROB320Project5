#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/TransformStamped.hpp"

namespace rix {
namespace msg {
namespace geometry {

class TF : public MessageBase {
  public:
    std::vector<geometry::TransformStamped> transforms;

    TF() = default;
    TF(const TF &other) = default;
    ~TF() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom_vec(transforms);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x1d1b0d204a1f70c0ULL, 0xb03dcbbdcf7ad206ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom_vec(transforms, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom_vec(transforms, buffer, offset);
    }
};

} // namespace geometry
} // namespace msg
} // namespace rix