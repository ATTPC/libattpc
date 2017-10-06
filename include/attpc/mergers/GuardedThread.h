#ifndef ATTPC_MERGERS_GUARDEDTHREAD_H
#define ATTPC_MERGERS_GUARDEDTHREAD_H

#include <thread>

namespace attpc {
namespace mergers {

class GuardedThread : public std::thread {
public:
    using std::thread::thread;
    ~GuardedThread() {
        if (joinable()) {
            join();
        }
    }
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GUARDEDTHREAD_H */
