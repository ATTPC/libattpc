#ifndef ATTPC_MERGERS_UTILITIES_H
#define ATTPC_MERGERS_UTILITIES_H

#include <cstddef>
#include <type_traits>
#include <iterator>
#include <cassert>

namespace attpc {
namespace mergers {
namespace utilities {

template <class T, class Iterator>
T parseValue(const Iterator& begin, const Iterator& end) {
    static_assert(!(std::is_integral<T>::value && std::is_signed<T>::value),
                  "Left bit shifts on signed integers can invoke undefined behavior, so this is not allowed.");
    assert(end > begin);

    const size_t numBytes = static_cast<size_t>(std::distance(begin, end));
    assert(numBytes <= sizeof(T));

    T result = 0;
    unsigned offset = 0;
    for (auto iter = end - 1; iter >= begin; --iter) {
        T byte = *iter;
        result |= (byte << offset*8u);
        ++offset;
    }
    return result;
}

}
}
}

#endif /* end of include guard: ATTPC_MERGERS_UTILITIES_H */
