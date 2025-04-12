#pragma once

#include "rix/core/common.hpp"
#include "rix/ipc/flags.hpp"

namespace rix {
namespace core {

template <typename TRequest, typename TResponse>
using TSrvCallback = std::function<void(const TRequest &, TResponse &)>;

class SrvCallbackBase {
   public:
    virtual void call(const ipc::Buffer &request, ipc::Buffer &response) = 0;
};

}  // namespace core
}  // namespace rix