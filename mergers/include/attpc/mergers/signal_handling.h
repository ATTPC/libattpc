#include <csignal>
/**
 * @file
 * @brief A basic signal handler for the merger.
 *
 * This implements a very basic signal handler that can catch a signal like SIGINT (usually issued by
 * control-C). To use it, assign ::abortHandler as a signal handler using std::signal, and then
 * later check whether the signal was received using ::abortWasCalled.
 *
 * If you improve this signal handler later, note that signal handlers can only perform an extremely
 * restricted set of operations according to the standard, and doing much more could be undefined behavior.
 */

/**
 * @brief A signal handler that sets a flag when it is called.
 *
 * Check the status of this flag with ::abortWasCalled.
 *
 * @param signal The signal number. This parameter is unused, but required by the standard.
 */
extern "C" void abortHandler(int signal);

/**
 * @brief Check on the status of the signal flag.
 * @return True if the signal was received and the program should abort.
 */
bool abortWasCalled();
