#ifndef ATTPC_MERGERS_RAWFRAME_H
#define ATTPC_MERGERS_RAWFRAME_H

#include <cstddef>
#include <memory>

namespace attpc {
namespace mergers {

class RawFrame {
public:
    using byte_type = uint8_t;
    using iterator = byte_type*;
    using const_iterator = const byte_type*;

    explicit RawFrame(const size_t size) : m_data(new byte_type[size]), m_size(size) {}

    iterator begin() { return begin_impl(); }
    iterator begin() const { return begin_impl(); }

    iterator end() { return end_impl(); }
    iterator end() const { return end_impl(); }

    const_iterator cbegin() const { return begin_impl(); }
    const_iterator cend() const { return end_impl(); }

    size_t size() const { return m_size; }

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
