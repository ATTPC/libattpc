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

FullTraceEvent Merger::buildEvent(const FrameAccumulator::FrameVector& frames) const {
    FullTraceEvent event {};
    event.setEventId(frames.front().header.eventIdx.value);
    event.setTimestamp(frames.front().header.eventTime.value);

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
                bool wasInserted;
                std::tie(traceIter, wasInserted) = event.insertTrace(Trace{addr});
                assert(wasInserted && (traceIter != event.end()));
            }

            (*traceIter)(item.timeBucket) = item.sample;
        }
    }

    return event;
}

}
}
