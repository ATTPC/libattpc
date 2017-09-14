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

    iterator begin() { return m_data.get(); }
    iterator begin() const { return m_data.get(); }

    iterator end() { return m_data.get() + m_size + 1; }
    iterator end() const { return m_data.get() + m_size + 1; }

    const_iterator cbegin() const { return m_data.get(); }
    const_iterator cend() const { return m_data.get() + m_size + 1; }

    size_t size() const { return m_size; }

    byte_type* getDataPtr() const { return m_data.get(); }

private:
    std::unique_ptr<byte_type> m_data;
    const size_t m_size;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_RAWFRAME_H */
