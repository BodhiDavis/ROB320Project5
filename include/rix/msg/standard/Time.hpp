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

class Time : public MessageBase {
  public:
    int32_t sec;
    int32_t nsec;

    Time() = default;
    Time(const Time &other) = default;
    ~Time() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(sec);
        size += size_base(nsec);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0x8c811e31b80f681bULL, 0xf4160d1e0e54d989ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(sec, buffer);
        serialize_base(nsec, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(sec, buffer, offset);
        deserialize_base(nsec, buffer, offset);
    }
};

} // namespace standard
} // namespace msg
} // namespace rix