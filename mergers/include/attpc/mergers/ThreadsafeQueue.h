#ifndef ATTPC_MERGERS_THREADSAFE_QUEUE_H
#define ATTPC_MERGERS_THREADSAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <exception>

namespace attpc {
namespace mergers {

template <class T>
class ThreadsafeQueue {
public:
    using task_type = T;

    ThreadsafeQueue() : finished(false) , maxSize(20) {}
    ThreadsafeQueue(size_t maxSize_) : finished(false), maxSize(maxSize_) {}
    ThreadsafeQueue(const ThreadsafeQueue&) = delete;
    ThreadsafeQueue(ThreadsafeQueue&&) = delete;

    void push(const task_type& task) {
        std::unique_lock<std::mutex> lock {queueMutex};
        condVar.wait(lock, [this](){ return contents.size() < maxSize; });
        contents.push(task);
        condVar.notify_all();
    }

    void push(task_type&& task) {
        std::unique_lock<std::mutex> lock {queueMutex};
        condVar.wait(lock, [this](){ return contents.size() < maxSize; });
        contents.push(std::move(task));
        condVar.notify_all();
    }

    task_type pop() {
        std::unique_lock<std::mutex> lock {queueMutex};
        condVar.wait(lock, [this](){ return !contents.empty() || finished; });
        if (finished) { throw QueueFinished(); }
        task_type item = std::move(contents.front());
        contents.pop();
        condVar.notify_all();
        return item;
    }

    void finish(bool waitUntilEmpty = true) {
        std::unique_lock<std::mutex> lock {queueMutex};
        if (waitUntilEmpty) {
            condVar.wait(lock, [this](){ return contents.empty(); });
        }
        finished = true;
        condVar.notify_all();
    }

    class QueueFinished : public std::exception {
    public:
        virtual const char* what() const noexcept override { return "Queue has been finished"; }
    };

private:
    std::queue<task_type> contents;
    std::mutex queueMutex;
    std::condition_variable condVar;
    bool finished;
    size_t maxSize;

};

}
}


#endif /* end of include guard: ATTPC_MERGERS_THREADSAFE_QUEUE_H */
