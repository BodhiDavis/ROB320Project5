#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace msg {
namespace sensor {

class JointState : public MessageBase {
  public:
    std::string name;
    double position;
    double velocity;
    double effort;

    JointState() = default;
    JointState(const JointState &other) = default;
    ~JointState() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_string(name);
        size += size_base(position);
        size += size_base(velocity);
        size += size_base(effort);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x2e219ddba0b464f9ULL, 0x969effb4b3ed37e9ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_string(name, buffer);
        serialize_base(position, buffer);
        serialize_base(velocity, buffer);
        serialize_base(effort, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_string(name, buffer, offset);
        deserialize_base(position, buffer, offset);
        deserialize_base(velocity, buffer, offset);
        deserialize_base(effort, buffer, offset);
    }
};

} // namespace sensor
} // namespace msg
} // namespace rix