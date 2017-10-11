#include "attpc/mergers/signal_handling.h"

/* This flag is set to 1 when the signal is received.
 * About the only thing that a signal handler can do is modify a static volatile variable
 * of this type, so that's what we do.
 */
static volatile std::sig_atomic_t abortFlag = 0;

extern "C" void abortHandler(int) {
    abortFlag = 1;
}

bool abortWasCalled() {
    return abortFlag != 0;
}
