#ifndef ATTPC_MERGERS_WORKER_H
#define ATTPC_MERGERS_WORKER_H

#include "attpc/mergers/ThreadsafeQueue.h"
#include "attpc/mergers/GuardedThread.h"
#include <memory>
#include <functional>

namespace attpc {
namespace mergers {

template <class T>
class Worker {
public:
    using task_type = T;
    using taskqueue_type = ThreadsafeQueue<task_type>;

    Worker(std::shared_ptr<taskqueue_type> taskQueuePtr)
    : tasks(std::move(taskQueuePtr))
    , thread(std::bind(&Worker::run, this))
    {}

    Worker(const Worker&) = delete;

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
