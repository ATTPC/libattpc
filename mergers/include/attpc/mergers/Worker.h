#ifndef ATTPC_MERGERS_WORKER_H
#define ATTPC_MERGERS_WORKER_H

#include "attpc/mergers/ThreadsafeQueue.h"
#include "attpc/mergers/GuardedThread.h"
#include <memory>
#include <functional>

namespace attpc {
namespace mergers {

/**
 * @brief A thread that performs tasks from a queue.
 *
 * On construction, this class spawns a thread that executes the Worker::run method. This method waits for tasks to
 * be pushed onto the task queue provided to the Worker, and then it executes them one-by-one by calling them.
 *
 * The tasks can be any type that provides a call operator (`operator()`), like a std::packaged_task, for example.
 * Nothing is done with the return value of the task, so any results should be handled within the task body.
 *
 * @tparam T The type of the tasks in the queue.
 */
template <class T>
class Worker {
public:
    //! The type of the tasks in the queue.
    using task_type = T;
    //! The type of the task queue.
    using taskqueue_type = ThreadsafeQueue<task_type>;

    /**
     * @brief Create a new worker thread that handles tasks in the given queue.
     *
     * The thread will be started upon construction and stopped upon destruction.
     *
     * @param taskQueuePtr Pointer to the task queue.
     */
    Worker(std::shared_ptr<taskqueue_type> taskQueuePtr)
    : tasks(std::move(taskQueuePtr))
    , thread(std::bind(&Worker::run, this))
    {}

    //! Workers cannot be copied.
    Worker(const Worker&) = delete;

    /**
     * @brief Loop that extracts tasks from the queue and runs them.
     *
     * This main loop can be stopped by calling ThreadsafeQueue::finish on the task queue. You should not
     * need to run this function explicitly since it is called by the constructor.
     */
    void run() {
        while (true) {
            task_type task;
            try {
                task = tasks->pop();
            }
            catch (const typename taskqueue_type::QueueFinished&) {
                return;
            }

            task();
        }
    }

private:
    std::shared_ptr<taskqueue_type> tasks;
    GuardedThread thread;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_WORKER_H */
