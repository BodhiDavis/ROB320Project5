#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"

namespace rix {
namespace msg {
namespace standard {

class String : public MessageBase {
  public:
    std::string data;

    String() = default;
    String(const String &other) = default;
    ~String() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_string(data);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xbf381bd7d9f0c5daULL, 0x575c82aa5ca40c09ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_string(data, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_string(data, buffer, offset);
    }
};

} // namespace standard
} // namespace msg
} // namespace rix