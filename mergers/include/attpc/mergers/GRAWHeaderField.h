#ifndef ATTPC_MERGERS_GRAWHEADERFIELD_H
#define ATTPC_MERGERS_GRAWHEADERFIELD_H

#include <cstddef>
#include "attpc/mergers/RawFrame.h"
#include "attpc/mergers/utilities.h"

namespace attpc {
namespace mergers {

/**
 * @brief A single field in the GRAWHeader, including type, offset, and size information.
 *
 * This class provides a way of encoding the offset, size, and type of each field in the
 * GRAWHeader directly in the field itself. Therefore, this information only has to be written
 * in one place in the library (in the GRAWHeader header file), making it easier to maintain.
 * In the future, this could also be read from a configuration file, though that would
 * prevent the offset and size from being known at compile time.
 *
 * Aside from the increased complexity, the one disadvantage of this approach is that you have
 * to access the value of the field through the member GRAWHeaderField::value instead of
 * using it directly.
 *
 * @tparam T       The type used to represent the value of the field
 * @tparam offset_ The offset from the beginning of the raw header to the encoded value
 * @tparam size_   The number of bytes that the field occupies in the raw header
 */
template <class T, size_t offset_, size_t size_ = 1>
struct GRAWHeaderField {
    static_assert(sizeof(T) >= size_, "Declared type is too small to hold a value with the given size");

    //! Construct an empty field
    GRAWHeaderField() = default;
    /**
     * @brief Construct a field using the value from the raw header
     *
     * The header is parsed using extractFromRawFrame. The value will be read using the offset
     * and size specified in the template parameters.
     *
     * @param rawFrame The raw frame from the GRAW file
     */
    GRAWHeaderField(const RawFrame& rawFrame) : value(extractFromRawFrame(rawFrame)) {}

    //! The type used to represent the field's value
    using type = T;
    //! The offset into the raw header where the value is encoded
    static constexpr size_t offset = offset_;
    //! The size of the encoded value in the header, in bytes
    static constexpr size_t size = size_;
    //! The value of the field in the header
    type value;

    /**
     * @brief Parse the raw frame to get the value of this field
     *
     * The value is read using the offset and size given as template parameters to the class.
     * Byte-swapping is performed automatically.
     *
     * @param  rawFrame The raw frame from the GRAW file.
     * @return          The value of this field in the header.
     *
     * @throws std::runtime_error If the end of the field (offset + size) is past the end of the raw frame
     */
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
