#include <csignal>

extern "C" void abortHandler(int signal);
bool abortWasCalled();
