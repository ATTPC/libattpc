#ifndef ATTPC_COMMON_UTILITIES
#define ATTPC_COMMON_UTILITIES

#include <stdexcept>

namespace attpc {
namespace common {

template <typename Output, typename Input>
Output narrow_cast(const Input value) {
    auto castValue = static_cast<Output>(value);
    if (static_cast<Input>(castValue) != value) {
        throw std::runtime_error("narrow_cast failed");
    }

    return castValue;
}

}
}


#endif /* end of include guard: ATTPC_COMMON_UTILITIES */
