#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <cstring>

#include "rix/msg/message_base.hpp"
#include "rix/msg/serializer.hpp"
#include "rix/msg/standard/Time.hpp"

namespace rix {
namespace msg {
namespace standard {

class Header : public MessageBase {
  public:
    uint32_t seq;
    standard::Time stamp;
    std::string frame_id;

    Header() = default;
    Header(const Header &other) = default;
    ~Header() = default;

  private:
    size_t size() const override {
        size_t size = 0;
        size += size_base(seq);
        size += size_custom(stamp);
        size += size_string(frame_id);
        return size;
    }

    rix::msg::Hash hash() const override {
        return {0xf5dd85dfcaa84a0eULL, 0x252e1b9e3dd24314ULL};
    }

    void serialize(std::vector<uint8_t> &buffer) const override {
        buffer.reserve(buffer.size() + this->size());
        serialize_base(seq, buffer);
        serialize_custom(stamp, buffer);
        serialize_string(frame_id, buffer);
    }

    void deserialize(const std::vector<uint8_t> &buffer, size_t &offset) override {
        deserialize_base(seq, buffer, offset);
        deserialize_custom(stamp, buffer, offset);
        deserialize_string(frame_id, buffer, offset);
    }
};

} // namespace standard
} // namespace msg
} // namespace rix