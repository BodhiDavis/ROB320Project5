#pragma once

#include "rix/ipc/impl/executor_impl.hpp"

namespace rix {
namespace ipc {

class Async;

class Executor {
  friend class Async;
  public:
    /**
     * @brief Callback type for a completion handler. These callbacks will be 
     * invoked after an asynchronous operation has finished.
     * 
     */
    typedef std::function<void()> CompletionHandler;

    static void post(const CompletionHandler &task, uint8_t priority=UINT8_MAX);
    static int run();
    static int run_once();
    static void shutdown();
    static void restart();
    static bool stopped();
    static size_t get_num_tasks();

  private:
    static void async_begin();

    static void async_end();
    // This must be called from within an asynchronous task (it cannot be within a task posted to the Executor).
    static void async_end(const std::function<void()> &task, uint8_t priority=UINT8_MAX);

    static ExecutorImpl impl;
};

}  // namespace ipc
}  // namespace rix