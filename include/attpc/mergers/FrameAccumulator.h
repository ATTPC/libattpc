#ifndef ATTPC_MERGERS_FRAMEACCUMULATOR_H
#define ATTPC_MERGERS_FRAMEACCUMULATOR_H

#include <vector>
#include <set>
#include <stdexcept>
#include "attpc/mergers/GRAWFrame.h"
#include "attpc/mergers/LRUQueue.h"

namespace attpc {
namespace mergers {

class FrameAccumulator {
public:
    using KeyType = uint64_t;
    using FrameVector = std::vector<GRAWFrame>;

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

    LRUQueue<KeyType, FrameVector> partialEvents;
    std::set<KeyType> finishedEvents;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_FRAMEACCUMULATOR_H */
