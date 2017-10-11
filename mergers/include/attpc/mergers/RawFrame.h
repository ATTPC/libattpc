#ifndef ATTPC_MERGERS_RAWFRAME_H
#define ATTPC_MERGERS_RAWFRAME_H

#include <cstddef>
#include <memory>

namespace attpc {
namespace mergers {

/**
 * @brief Container for raw frame data read from a GRAWFile.
 *
 * This holds a fixed-size block of memory and provides iterators into it. That is all.
 */
class RawFrame {
public:
    //! The type of the contained data
    using byte_type = uint8_t;
    //! An iterator into the data
    using iterator = byte_type*;
    //! A constant iterator into the data
    using const_iterator = const byte_type*;

    /**
     * @brief Construct a new RawFrame buffer
     * @param  size The number of bytes to allocate
     */
    explicit RawFrame(const size_t size) : m_data(new byte_type[size]), m_size(size) {}

    //! Get an iterator to the beginning of the data
    iterator begin() { return begin_impl(); }
    //! @overload
    iterator begin() const { return begin_impl(); }

    //! Get an iterator to the end of the data
    iterator end() { return end_impl(); }
    //! @overload
    iterator end() const { return end_impl(); }

    //! Get a constant iterator to the beginning of the data
    const_iterator cbegin() const { return begin_impl(); }
    //! Get a constant iterator to the end of the data
    const_iterator cend() const { return end_impl(); }

    //! Returns the size of the data, in bytes
    size_t size() const { return m_size; }

    //! Gets a pointer to the raw data
    byte_type* getDataPtr() const { return m_data.get(); }

private:
    inline iterator begin_impl() const { return m_data.get(); }
    inline iterator end_impl() const { return m_data.get() + m_size; }

    std::unique_ptr<byte_type> m_data;
    const size_t m_size;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_RAWFRAME_H */
