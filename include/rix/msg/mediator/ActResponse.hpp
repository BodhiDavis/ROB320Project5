#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/ActInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class ActResponse : public MessageBase {
  public:
    uint8_t error;
    mediator::ActInfo act_info;

    ActResponse() = default;
    ActResponse(const ActResponse &other) = default;
    ~ActResponse() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(error);
        size += size_custom(act_info);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x48251511dd63a305ULL, 0xa05d95bab2f31716ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(error, buffer);
        serialize_custom(act_info, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(error, buffer, offset);
        deserialize_custom(act_info, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix