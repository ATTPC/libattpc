#ifndef ATTPC_COMMON_FULLTRACEEVENT_H
#define ATTPC_COMMON_FULLTRACEEVENT_H

#include <map>
#include <iterator>
#include <type_traits>
#include "attpc/common/Trace.h"
#include "attpc/common/HardwareAddress.h"

namespace attpc {
namespace common {

class FullTraceEvent {
public:
    // using iterator = std::map<HardwareAddress, Trace>::iterator;
    // using const_iterator = std::map<HardwareAddress, Trace>::const_iterator;

    template <class BaseIterator, bool isConst>
    class MapIteratorAdapter {
    public:
        using iterator_category = typename std::iterator_traits<BaseIterator>::iterator_category;
        // using value_type = typename std::iterator_traits<BaseIterator>::value_type::second_type;
        using value_type = typename std::conditional<
            isConst,
            typename std::add_const<typename std::iterator_traits<BaseIterator>::value_type::second_type>::type,
            typename std::iterator_traits<BaseIterator>::value_type::second_type
            >::type;
        using difference_type = typename std::iterator_traits<BaseIterator>::difference_type;
        using pointer = value_type*;
        using reference = value_type&;

        MapIteratorAdapter() = default;
        MapIteratorAdapter(BaseIterator&& baseIter) : base(baseIter) {}

        bool operator==(const MapIteratorAdapter& other) { return base == other.base; }
        bool operator!=(const MapIteratorAdapter& other) { return base != other.base; }

        reference operator*() { return (*base).second; }
        pointer operator->() { return &(base->second); }
        MapIteratorAdapter& operator++() {
            ++base;
            return *this;
        }
        MapIteratorAdapter operator++(int) {
            MapIteratorAdapter result = *this;
            ++(*this);
            return result;
        }
        MapIteratorAdapter& operator--() {
            --base;
            return *this;
        }
        MapIteratorAdapter operator--(int) {
            MapIteratorAdapter result = *this;
            --(*this);
            return result;
        }

    private:
        BaseIterator base;
    };

    using iterator = MapIteratorAdapter<std::map<HardwareAddress, Trace>::iterator, false>;
    using const_iterator = MapIteratorAdapter<std::map<HardwareAddress, Trace>::const_iterator, true>;

    FullTraceEvent();
    FullTraceEvent(const evtid_type eventId_, const timestamp_type timestamp_);

    std::tuple<iterator, bool> insertTrace(const Trace& trace);
    std::tuple<iterator, bool> insertTrace(Trace&& trace);

    iterator insertOrReplaceTrace(const Trace& trace);
    iterator insertOrReplaceTrace(Trace&& trace);

    iterator findTrace(const HardwareAddress& hwaddr);
    const_iterator findTrace(const HardwareAddress& hwaddr) const;

    iterator begin() noexcept { return traces.begin(); }
    const_iterator begin() const noexcept { return traces.begin(); }
    const_iterator cbegin() const noexcept { return traces.cbegin(); }

    iterator end() noexcept { return traces.end(); }
    const_iterator end() const noexcept { return traces.end(); }
    const_iterator cend() const noexcept { return traces.cend(); }

    size_t numTraces() const { return traces.size(); }

    evtid_type getEventId() const { return eventId; }
    void setEventId(const evtid_type newEventId) { eventId = newEventId; }

    timestamp_type getTimestamp() const { return timestamp; }
    void setTimestamp(const timestamp_type newTimestamp) { timestamp = newTimestamp; }

    bool operator==(const FullTraceEvent& other) const;

private:
    evtid_type eventId;
    timestamp_type timestamp;
    std::map<HardwareAddress, Trace> traces;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_FULLTRACEEVENT_H */
