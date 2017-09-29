#ifndef ATTPC_MERGERS_MERGER
#define ATTPC_MERGERS_MERGER

#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/FullTraceEvent.h"

namespace attpc {
namespace mergers {


class Merger {
public:
    attpc::common::FullTraceEvent buildEvent(const FrameAccumulator::FrameVector& frames) const;

};

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGER */
