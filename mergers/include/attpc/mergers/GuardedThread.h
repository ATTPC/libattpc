#ifndef ATTPC_MERGERS_GUARDEDTHREAD_H
#define ATTPC_MERGERS_GUARDEDTHREAD_H

#include <thread>

namespace attpc {
namespace mergers {

/**
 * @brief A thread that is automatically joined upon destruction.
 *
 * Before destroying a thread, one should call std::thread::join to wait for the thread
 * to finish whatever it's doing. This subclass of std::thread does that automatically upon
 * destruction, making it a safer alternative to the std::thread class. This way, a GuardedThread
 * will automatically and safely be joined and terminated when it goes out of scope.
 */
class GuardedThread : public std::thread {
public:
    using std::thread::thread;

    //! Destroy the thread after joining it
    virtual ~GuardedThread() {
        if (joinable()) {
            join();
        }
    }
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GUARDEDTHREAD_H */
