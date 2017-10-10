#ifndef ATTPC_MERGERS_GRAWHEADERFIELD_H
#define ATTPC_MERGERS_GRAWHEADERFIELD_H

#include <cstddef>
#include "attpc/mergers/RawFrame.h"
#include "attpc/mergers/utilities.h"

namespace attpc {
namespace mergers {

template <class T, size_t offset_, size_t size_ = 1>
struct GRAWHeaderField {
    static_assert(sizeof(T) >= size_, "Declared type is too small to hold a value with the given size");

    GRAWHeaderField() = default;
    GRAWHeaderField(const RawFrame& rawFrame) : value(extractFromRawFrame(rawFrame)) {}

    using type = T;
    static constexpr size_t offset = offset_;
    static constexpr size_t size = size_;
    type value;

    static type extractFromRawFrame(const RawFrame& rawFrame) {
        const auto fieldBegin = rawFrame.begin() + offset;
        const auto fieldEnd = fieldBegin + size;
        if (fieldEnd > rawFrame.end()) {
            throw std::runtime_error("Header field was past the end of the frame.");
        }
        return utilities::parseValue<type>(fieldBegin, fieldEnd);
    }
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWHEADERFIELD_H */
