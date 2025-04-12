#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace msg {
namespace mediator {

class Endpoint : public MessageBase {
  public:
    uint16_t port;
    std::string address;

    Endpoint() = default;
    Endpoint(const Endpoint &other) = default;
    ~Endpoint() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(port);
        size += size_string(address);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xc972b452ee36a39cULL, 0x2c279f8c99e51b82ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(port, buffer);
        serialize_string(address, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(port, buffer, offset);
        deserialize_string(address, buffer, offset);
    }
};

} // namespace mediator
} // namespace msg
} // namespace rix