#ifndef ATTPC_MERGERS_MERGER
#define ATTPC_MERGERS_MERGER

#include "attpc/mergers/GRAWFrame.h"
#include "attpc/mergers/LRUCache.h"
#include <vector>
#include <functional>
#include "attpc/common/FullTraceEvent.h"

namespace attpc {
namespace mergers {


class Merger {
public:
    using KeyType = uint64_t;
    using FrameVector = std::vector<GRAWFrame>;

    void addFrame(const GRAWFrame& frame);
    void addFrame(GRAWFrame&& frame);

    attpc::common::FullTraceEvent buildEvent(const FrameVector& frames) const;



private:
    LRUCache<KeyType, FrameVector> partialEventCache;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGER */
