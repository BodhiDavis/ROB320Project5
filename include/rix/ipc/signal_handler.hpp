#pragma once

#include <stdio.h> 
#include <stdlib.h> 
#include <sys/types.h> 
#include <unistd.h> 
#include <signal.h>

#include <unordered_map>
#include <functional>
#include <vector>

namespace rix {
namespace ipc {

class SignalHandler {
   public:
    using HandlerFunction = std::function<void(int)>;

    static void register_handle(int signal, HandlerFunction handler);
    static void handle(int signal);

   private:
    static std::unordered_multimap<int, HandlerFunction> handlers;  ///< Map of signal handlers.
};

}  // namespace ipc
}  // namespace rix