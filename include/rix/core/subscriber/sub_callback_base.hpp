#pragma once

#include "rix/core/common.hpp"
#include "rix/ipc/flags.hpp"

namespace rix {
namespace core {

template <typename TMsg>
using TSubCallback = std::function<void(const TMsg&)>;

class SubCallbackBase {
   public:
    virtual void call(const ipc::Buffer &buffer) = 0;
};

}  // namespace core
}  // namespace rix