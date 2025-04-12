#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/geometry/Transform.hpp"
#include "rix/msg/sensor/JS.hpp"

namespace rix {
namespace msg {
namespace iksrv {

class IKRequest : public MessageBase {
  public:
    std::string end_effector;
    geometry::Transform goal;
    sensor::JS initial_guess;
    double step_scale;
    double tolerance;
    uint32_t max_iterations;
    bool debug;

    IKRequest() = default;
    IKRequest(const IKRequest &other) = default;
    ~IKRequest() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_string(end_effector);
        size += size_custom(goal);
        size += size_custom(initial_guess);
        size += size_base(step_scale);
        size += size_base(tolerance);
        size += size_base(max_iterations);
        size += size_base(debug);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x5e0a5bff54a7cf36ULL, 0xdf47a4b7953bf517ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_string(end_effector, buffer);
        serialize_custom(goal, buffer);
        serialize_custom(initial_guess, buffer);
        serialize_base(step_scale, buffer);
        serialize_base(tolerance, buffer);
        serialize_base(max_iterations, buffer);
        serialize_base(debug, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_string(end_effector, buffer, offset);
        deserialize_custom(goal, buffer, offset);
        deserialize_custom(initial_guess, buffer, offset);
        deserialize_base(step_scale, buffer, offset);
        deserialize_base(tolerance, buffer, offset);
        deserialize_base(max_iterations, buffer, offset);
        deserialize_base(debug, buffer, offset);
    }
};

} // namespace iksrv
} // namespace msg
} // namespace rix