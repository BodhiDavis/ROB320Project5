#pragma once

#include <pthread.h>

#include <unordered_map>

#include "rix/ipc/executor.hpp"

namespace rix {
namespace ipc {

class Async {
   public:
    /**
     * @brief Callback type for an asynchronous operation. These callbacks will
     * not be invoked by the context, but in background threads. This type is
     * specified here for clarity.
     *
     */
    typedef std::function<void()> AsyncOperation;

    /**
     * @brief Start a thread in the background. This function will increment
     * the Executor's `async_outstanding` member variable so that the
     * Executor knows that there are unfinished asynchronous tasks.
     *
     * @warning This function must call `Async::post` exactly once.
     *
     * @param task The function to be invoked in a background thread.
     * @returns The ID of the async operation
     */
    static std::thread::id start(const AsyncOperation &task);

    /**
     * @brief Enqueue a task to the Executor exectuor. This function will
     * decrement the Executor's `async_outstanding` member variable before
     * enqueueing the task.
     *
     * @warning This function should only be used from within
     * the `AsyncOperation` specified in `Async::start`.
     *
     * @param tid The thread ID of the asynchronous operation invoking post.
     * This function call will join this thread.
     * @param task The callback function to be invoked by the executor.
     */
    static int end(const std::thread::id &tid, const Executor::CompletionHandler &task, uint8_t priority = UINT8_MAX);
    static int end(const std::thread::id &tid);

    static int cancel(const std::thread::id &tid);
    static void cancel_all();
    static bool cancelled(const std::thread::id &tid);
    static void join(const std::thread::id &tid);
    static void join_all();

   private:
    static void async_completion_handler(const std::thread::id &tid, const Executor::CompletionHandler &task);

    struct ThreadInfo {
        std::shared_ptr<std::thread> thread;
        std::atomic<bool> cancelled;

        ThreadInfo(std::shared_ptr<std::thread> thread, bool cancelled) : thread(thread), cancelled(cancelled) {}
        ThreadInfo() {}
    };

    struct ThreadMap {
        std::unordered_map<std::thread::id, ThreadInfo> map;
        ~ThreadMap() {
            for (auto &it : map) {
                it.second.cancelled = true;
                it.second.thread->join();
            }
        }
    };

    // Ensures that the threads mutex stays alive for the duration of the program.
    static std::mutex &get_threads_mutex() {
        static std::mutex threads_mutex;
        return threads_mutex;
    }

    // Ensures that the threads map stays alive for the duration of the program.
    static ThreadMap &get_threads() {
        static ThreadMap threads;
        return threads;
    }
};

}  // namespace ipc
}  // namespace rix