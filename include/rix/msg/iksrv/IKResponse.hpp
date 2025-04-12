#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/sensor/JS.hpp"

namespace rix {
namespace msg {
namespace iksrv {

class IKResponse : public MessageBase {
  public:
    sensor::JS result;
    bool converged;
    bool debug;
    std::vector<sensor::JS> steps;

    IKResponse() = default;
    IKResponse(const IKResponse &other) = default;
    ~IKResponse() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(result);
        size += size_base(converged);
        size += size_base(debug);
        size += size_custom_vec(steps);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x7e0be5cebe66c1baULL, 0x6022bd17b3f674a2ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(result, buffer);
        serialize_base(converged, buffer);
        serialize_base(debug, buffer);
        serialize_custom_vec(steps, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(result, buffer, offset);
        deserialize_base(converged, buffer, offset);
        deserialize_base(debug, buffer, offset);
        deserialize_custom_vec(steps, buffer, offset);
    }
};

} // namespace iksrv
} // namespace msg
} // namespace rix