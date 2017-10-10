#ifndef ATTPC_COMMON_UTILITIES
#define ATTPC_COMMON_UTILITIES

#include <stdexcept>

namespace attpc {
namespace common {

/**
 * @brief Safely convert one integer type to another.
 *
 * This will cast an integer to a different type, failing if the result does not equal the original value.
 * @param  value The integer to cast.
 * @return       The result.
 */
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
