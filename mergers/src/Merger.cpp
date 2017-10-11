#include "attpc/mergers/Merger.h"
#include "attpc/common/utilities.h"
#include <tuple>
#include <cassert>

namespace {
using attpc::common::FullTraceEvent;
using attpc::common::HardwareAddress;
using attpc::common::Trace;
}

namespace attpc {
namespace mergers {

const std::vector<channelid_type> Merger::fpnChannels = {11, 22, 45, 56};

Merger::Merger()
: lookupPtr(nullptr)
, keepFPN(false)
{}

Merger::Merger(std::shared_ptr<common::PadLookupTable> lookupPtr_, bool keepFPN_)
: lookupPtr(std::move(lookupPtr_))
, keepFPN(keepFPN_)
{}

FullTraceEvent Merger::mergeAndProcessEvent(const FrameAccumulator::FrameVector& frames) const {
    common::FullTraceEvent event = mergeFrames(frames);
    if (!keepFPN) {
        discardFPN(event);
    }
    if (lookupPtr) {
        setPadNumbers(event);
    }
    return event;
}

FullTraceEvent Merger::mergeFrames(const FrameAccumulator::FrameVector& frames) const {
    FullTraceEvent event {};
    // NOTE: This assumes that the frames all have the same event ID and timestamp
    event.setEventId(frames.front().getEventId());
    event.setTimestamp(frames.front().getTimestamp());

    for (const GRAWFrame& frame : frames) {
        for (const GRAWDataItem& item : frame.data) {
            HardwareAddress addr {
                common::narrow_cast<coboid_type>(frame.header.coboIdx.value),
                common::narrow_cast<asadid_type>(frame.header.asadIdx.value),
                common::narrow_cast<agetid_type>(item.aget),
                common::narrow_cast<channelid_type>(item.channel)
            };
            FullTraceEvent::iterator traceIter = event.findTrace(addr);
            if (traceIter == event.end()) {
                // In this case, the trace was not yet present in the event, so we have to create it
                bool wasInserted;
                std::tie(traceIter, wasInserted) = event.insertTrace(Trace{addr});
                assert(wasInserted && (traceIter != event.end()));
            }

            (*traceIter)(item.timeBucket) = item.sample;  // calls operator() on the Trace
        }
    }

    return event;
}

void Merger::discardFPN(common::FullTraceEvent& event) const {
    auto newEnd = std::remove_if(event.begin(), event.end(), [](const common::Trace& tr) {
        // Predicate returns True if channel number is in the list of FPN channels.
        auto begin = Merger::fpnChannels.begin();
        auto end = Merger::fpnChannels.end();
        return std::find(begin, end, tr.getHardwareAddress().channel) != end;
    });
    event.erase(newEnd, event.end());
}

void Merger::setPadNumbers(common::FullTraceEvent& event) const {
    if (lookupPtr) {
        for (common::Trace& trace : event) {
            trace.setPad(lookupPtr->find(trace.getHardwareAddress()));
        }
    }
}

}
}
