#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/sensor/JointState.hpp"
#include "rix/msg/standard/Time.hpp"

namespace rix {
namespace msg {
namespace sensor {

class JS : public MessageBase {
  public:
    standard::Time stamp;
    std::vector<sensor::JointState> joint_states;

    JS() = default;
    JS(const JS &other) = default;
    ~JS() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_custom(stamp);
        size += size_custom_vec(joint_states);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x63294d5423e692b9ULL, 0x24b04cf6dc45332eULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_custom(stamp, buffer);
        serialize_custom_vec(joint_states, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_custom(stamp, buffer, offset);
        deserialize_custom_vec(joint_states, buffer, offset);
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix