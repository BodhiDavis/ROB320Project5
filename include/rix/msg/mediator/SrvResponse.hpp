#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/mediator/SrvInfo.hpp"

namespace rix {
namespace msg {
namespace mediator {

class SrvResponse : public MessageBase {
  public:
    uint8_t error;
    mediator::SrvInfo srv_info;

    SrvResponse() = default;
    SrvResponse(const SrvResponse &other) = default;
    ~SrvResponse() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(error);
        size += size_custom(srv_info);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x63f8fc7338391845ULL, 0x5b640e4603b81f7eULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(error, buffer);
        serialize_custom(srv_info, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(error, buffer, offset);
        deserialize_custom(srv_info, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix