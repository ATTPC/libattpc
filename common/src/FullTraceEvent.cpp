#include "attpc/common/FullTraceEvent.h"
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
    return traces.emplace(trace.getHardwareAddress(), trace);
}

auto FullTraceEvent::insertTrace(Trace&& trace) -> std::tuple<iterator, bool> {
    HardwareAddress addr = trace.getHardwareAddress();
    return traces.emplace(addr, std::move(trace));
}

auto FullTraceEvent::insertOrReplaceTrace(const Trace& trace) -> iterator {
    decltype(traces)::iterator insertLoc;
    bool wasInserted;
    std::tie(insertLoc, wasInserted) = traces.emplace(trace.getHardwareAddress(), trace);
    if (!wasInserted) {
        traces.erase(insertLoc);
        std::tie(insertLoc, wasInserted) = traces.emplace(trace.getHardwareAddress(), trace);;
        assert(wasInserted);
    }
    return insertLoc;
}

auto FullTraceEvent::insertOrReplaceTrace(Trace&& trace) -> iterator {
    decltype(traces)::iterator insertLoc;
    bool wasInserted;
    HardwareAddress addr = trace.getHardwareAddress();
    std::tie(insertLoc, wasInserted) = traces.emplace(addr, std::move(trace));
    if (!wasInserted) {
        traces.erase(insertLoc);
        std::tie(insertLoc, wasInserted) = traces.emplace(addr, std::move(trace));
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

bool FullTraceEvent::operator==(const FullTraceEvent& other) const {
    return (eventId == other.eventId) && (timestamp == other.timestamp) && (traces == other.traces);
}


}
}
