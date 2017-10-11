#ifndef ATTPC_MERGERS_THREADSAFE_QUEUE_H
#define ATTPC_MERGERS_THREADSAFE_QUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <exception>

namespace attpc {
namespace mergers {

/**
 * @brief A queue that can be safely accessed from multiple threads.
 *
 * This can be used to feed tasks to a worker thread, for example. It protects itself from multithreaded
 * access using locks, so it's not the most efficient solution, but it's relatively simple.
 *
 * @tparam T The type of item that the queue will contain
 */
template <class T>
class ThreadsafeQueue {
public:
    //! The type of the contained items
    using task_type = T;

    //! Construct an empty queue with a maximum size of 20
    ThreadsafeQueue() : finished(false) , maxSize(20) {}
    /**
     * @brief Construct an empty queue with the given maximum size.
     * @param maxSize_ The maximum size of the queue.
     */
    ThreadsafeQueue(size_t maxSize_) : finished(false), maxSize(maxSize_) {}
    //! ThreadsafeQueues cannot be copied
    ThreadsafeQueue(const ThreadsafeQueue&) = delete;
    //! ThreadsafeQueues cannot be moved
    ThreadsafeQueue(ThreadsafeQueue&&) = delete;

    /**
     * @brief Push a new item onto the queue.
     *
     * This locks the queue while it executes. If the queue is full, this function
     * will block until the queue is no longer full.
     *
     * @param task The item to be pushed onto the queue.
     */
    void push(const task_type& task) {
        std::unique_lock<std::mutex> lock {queueMutex};
        condVar.wait(lock, [this](){ return contents.size() < maxSize; });
        contents.push(task);
        condVar.notify_all();
    }
    //! @overload
    void push(task_type&& task) {
        std::unique_lock<std::mutex> lock {queueMutex};
        condVar.wait(lock, [this](){ return contents.size() < maxSize; });
        contents.push(std::move(task));
        condVar.notify_all();
    }

    /**
     * @brief Remove the item at the front of the queue.
     *
     * This locks the queue while it executes. If the queue is empty, this function
     * will block until there is something in the queue or ThreadsafeQueue::finish is called.
     *
     * If the queue is finished using ThreadsafeQueue::finish and a thread is waiting in this
     * function, the waiting thread will unblock and throw QueueFinished.
     *
     * @return The item at the front of the queue.
     *
     * @throws QueueFinished If ThreadsafeQueue::finish has been called.
     */
    task_type pop() {
        std::unique_lock<std::mutex> lock {queueMutex};
        condVar.wait(lock, [this](){ return !contents.empty() || finished; });
        if (finished) { throw QueueFinished(); }
        task_type item = std::move(contents.front());
        contents.pop();
        condVar.notify_all();
        return item;
    }

    /**
     * @brief Notifies all users of the queue that no more tasks will be added.
     *
     * After calling this, all calls to ThreadsafeQueue::pop will throw QueueFinished. This
     * can be caught in a worker thread to tell it to shut down.
     *
     * @param waitUntilEmpty If true, wait until the queue is empty before marking it as finished.
     */
    void finish(bool waitUntilEmpty = true) {
        std::unique_lock<std::mutex> lock {queueMutex};
        if (waitUntilEmpty) {
            condVar.wait(lock, [this](){ return contents.empty(); });
        }
        finished = true;
        condVar.notify_all();
    }

    //! Indicates that the queue will not have more items added to it.
    class QueueFinished : public std::exception {
    public:
        virtual const char* what() const noexcept override { return "Queue has been finished"; }
    };

private:
    //! The contents of the queue.
    std::queue<task_type> contents;
    //! Mutex to protect access to the queue contents.
    std::mutex queueMutex;
    //! Allows threads to block until a condition having to do with the queue contents is satisfied.
    std::condition_variable condVar;
    //! If true, all calls to ThreadsafeQueue::pop will throw QueueFinished.
    bool finished;
    //! The maximum size of the queue.
    size_t maxSize;

};

}
}


#endif /* end of include guard: ATTPC_MERGERS_THREADSAFE_QUEUE_H */
