#ifndef ATTPC_COMMON_FULLTRACEEVENT_H
#define ATTPC_COMMON_FULLTRACEEVENT_H

#include <set>
#include "Trace.h"
#include "HardwareAddress.h"

namespace attpc {
namespace common {

class FullTraceEvent {
public:
    using iterator = std::set<Trace>::iterator;
    using const_iterator = std::set<Trace>::const_iterator;

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
    std::set<Trace> traces;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_FULLTRACEEVENT_H */
