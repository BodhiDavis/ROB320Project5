#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>

namespace rix {
namespace ipc {

class ExecutorImpl {
   public:
    ExecutorImpl();
    ~ExecutorImpl();
    ExecutorImpl(const ExecutorImpl &other) = delete;
    ExecutorImpl &operator=(const ExecutorImpl &other) = delete;

    // Post a task to the context. This will be executed in the context's thread.
    void post(const std::function<void()> &task, uint8_t priority=UINT8_MAX);

    // Blocking call that will dequeue tasks and execute them until the queue is empty and there are no outstanding
    // async tasks. Returns the number of tasks completed.
    int run();
    int run_once();

    // Stop the dequeuing tasks. Wait for outstanding async tasks to complete.
    void shutdown();

    // Clear the task queue.
    void restart();

    bool stopped() const;

    size_t get_num_tasks() const;

    void async_begin();
    void async_end();
    void async_end(const std::function<void()> &task, uint8_t priority=UINT8_MAX);

   private:

    struct Task {
        uint8_t priority;
        std::function<void()> task;

        bool operator<(const Task &other) const { return priority < other.priority; }
    };

    mutable std::mutex tasks_mutex;
    std::priority_queue<Task> tasks;
    int async_outstanding;
    
    std::condition_variable cv;
    std::atomic<bool> ready;
    std::atomic<bool> _stopped;
};

}  // namespace ipc
}  // namespace rix