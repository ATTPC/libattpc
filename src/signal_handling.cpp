#include "attpc/mergers/signal_handling.h"

static volatile std::sig_atomic_t abortFlag = 0;

extern "C" void abortHandler(int) {
    abortFlag = 1;
}

bool abortWasCalled() {
    return abortFlag != 0;
}
