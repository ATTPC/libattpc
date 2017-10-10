#ifndef ATTPC_MERGERS_MERGER
#define ATTPC_MERGERS_MERGER

#include <memory>
#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/FullTraceEvent.h"
#include "attpc/common/PadLookupTable.h"

namespace attpc {
namespace mergers {


class Merger {
public:
    Merger();
    Merger(std::shared_ptr<common::PadLookupTable> lookupPtr_, bool keepFPN_ = false);

    attpc::common::FullTraceEvent mergeAndProcessEvent(const FrameAccumulator::FrameVector& frames) const;
    attpc::common::FullTraceEvent mergeFrames(const FrameAccumulator::FrameVector& frames) const;
    void discardFPN(common::FullTraceEvent& event) const;
    void setPadNumbers(common::FullTraceEvent& event) const;

private:
    std::shared_ptr<common::PadLookupTable> lookupPtr;
    static const std::vector<channelid_type> fpnChannels;
    bool keepFPN;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGER */
