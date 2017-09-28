#include "attpc/mergers/Merger.h"
#include "attpc/common/utilities.h"

namespace {
using attpc::common::FullTraceEvent;
using attpc::common::HardwareAddress;
using attpc::common::Trace;
}

namespace attpc {
namespace mergers {

FullTraceEvent Merger::buildEvent(const FrameVector& frames) const {
    FullTraceEvent event {};
    event.setEventId(frames.front().header.eventIdx.value);
    event.setTimestamp(frames.front().header.eventTime.value);

    for (const GRAWFrame& frame : frames) {
        for (const GRAWDataItem& item : frame.data) {
            // HardwareAddress addr {
            //     frame.header.coboIdx.value,
            //     frame.header.asadIdx.value,
            //     item.aget,
            //     item.channel
            // };
        }
    }
}

}
}
