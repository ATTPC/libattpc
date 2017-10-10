#ifndef ATTPC_MERGERS_FRAMEACCUMULATOR_H
#define ATTPC_MERGERS_FRAMEACCUMULATOR_H

#include <vector>
#include <set>
#include <stdexcept>
#include <functional>
#include "attpc/mergers/GRAWFrame.h"
#include "attpc/mergers/LRUQueue.h"

namespace attpc {
namespace mergers {

/**
 * @brief A method for merging events.
 *
 * This type describes a function that can extract a merging key from a GRAWFrame. Two examples are defined:
 * - GetMergeKeyFromEventId: returns the event ID
 * - GetMergeKeyFromTimestamp: returns the timestamp
 * The choice between these two functions controls whether the events will be merged based on event ID or based
 * on timestamp.
 */
using MergeKeyFunction = std::function<uint64_t(const GRAWFrame&)>;
//! Returns the event ID, to merge based on event ID
uint64_t GetMergeKeyFromEventId(const GRAWFrame& frame);
//! Returns the event timestamp, to merge based on timestamp
uint64_t GetMergeKeyFromTimestamp(const GRAWFrame& frame);

/**
 * @brief Collects and sorts GRAW frames based on a merge key (i.e. event ID or timestamp)
 *
 * This class does the actual "merging," in some sense. It maintains a cache of vectors of frames (FrameVector)
 */
class FrameAccumulator {
public:
    using KeyType = uint64_t;
    using FrameVector = std::vector<GRAWFrame>;

    FrameAccumulator(MergeKeyFunction method) : getMergeKey(method) {}

    void addFrame(const GRAWFrame& frame);
    void addFrame(GRAWFrame&& frame);

    std::pair<KeyType, FrameVector> extractOldest();

    size_t size() const { return partialEvents.size(); }

    class EventAlreadyFinishedError : public std::runtime_error {
    public:
        EventAlreadyFinishedError(KeyType key)
            : std::runtime_error("Event " + std::to_string(key) + " was already finished.") {}
    };

private:
    FrameVector& getFrameVector(KeyType key);
    bool eventWasAlreadyFinished(KeyType key) const;
    MergeKeyFunction getMergeKey;

    LRUQueue<KeyType, FrameVector> partialEvents;
    std::set<KeyType> finishedEvents;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_FRAMEACCUMULATOR_H */
