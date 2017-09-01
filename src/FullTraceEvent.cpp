#include "FullTraceEvent.h"
#include <cassert>

namespace attpc {
namespace common {

FullTraceEvent::FullTraceEvent()
: eventId(0)
, timestamp(0)
{}

FullTraceEvent::FullTraceEvent(const evtid_type eventId_, const timestamp_type timestamp_)
: eventId(eventId_)
, timestamp(timestamp_)
{}

auto FullTraceEvent::insertTrace(const Trace& trace) -> std::tuple<iterator, bool> {
    return traces.insert(trace);
}

auto FullTraceEvent::insertTrace(Trace&& trace) -> std::tuple<iterator, bool> {
    return traces.insert(trace);
}

auto FullTraceEvent::insertOrReplaceTrace(const Trace& trace) -> iterator {
    iterator insertLoc;
    bool wasInserted;
    std::tie(insertLoc, wasInserted) = traces.insert(trace);
    if (!wasInserted) {
        traces.erase(insertLoc);
        std::tie(insertLoc, wasInserted) = traces.insert(trace);
        assert(wasInserted);
    }
    return insertLoc;
}

auto FullTraceEvent::insertOrReplaceTrace(Trace&& trace) -> iterator {
    iterator insertLoc;
    bool wasInserted;
    std::tie(insertLoc, wasInserted) = traces.insert(trace);
    if (!wasInserted) {
        traces.erase(insertLoc);
        std::tie(insertLoc, wasInserted) = traces.insert(trace);
        assert(wasInserted);
    }
    return insertLoc;
}

auto FullTraceEvent::findTrace(const HardwareAddress& hwaddr) -> iterator {
    return traces.find(hwaddr);
}

auto FullTraceEvent::findTrace(const HardwareAddress& hwaddr) const -> const_iterator {
    return traces.find(hwaddr);
}



}
}
