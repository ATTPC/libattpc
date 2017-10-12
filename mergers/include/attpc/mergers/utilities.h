#ifndef ATTPC_MERGERS_UTILITIES_H
#define ATTPC_MERGERS_UTILITIES_H

/**
 * @file
 * @brief Utilities for the merging code.
 */

#include <cstddef>
#include <type_traits>
#include <iterator>
#include <cassert>

namespace attpc {
namespace mergers {
namespace utilities {

/**
 * @brief Construct and byte-swap a value from a collection of bytes.
 *
 * This takes a collection of bytes (from a std::vector<uint8_t>, for example) and puts them together into
 * a byte-swapped integer. The returned value is of type `T`, where `T` must not be a signed integer since
 * that would invoke undefined behavior (by using the left-shift operator).
 *
 * @tparam T        The type of value to construct as a result.
 * @tparam Iterator The type of the begin and end iterators.
 *
 * @param  begin Iterator to the first byte.
 * @param  end   Iterator to the element past the last byte.
 *
 * @return       The byte-swapped value.
 */
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
