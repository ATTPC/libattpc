#include "attpc/mergers/FrameAccumulator.h"
#include <boost/optional.hpp>

namespace attpc {
namespace mergers {

uint64_t GetMergeKeyFromEventId(const GRAWFrame& frame) { return frame.getEventId(); }
uint64_t GetMergeKeyFromTimestamp(const GRAWFrame& frame) { return frame.getTimestamp(); }

void FrameAccumulator::addFrame(const GRAWFrame& frame) {
    KeyType eventKey = getMergeKey(frame);
    FrameVector& frvec = getFrameVector(eventKey);
    frvec.push_back(frame);
}

void FrameAccumulator::addFrame(GRAWFrame&& frame) {
    KeyType eventKey = getMergeKey(frame);
    FrameVector& frvec = getFrameVector(eventKey);
    frvec.push_back(frame);
}

auto FrameAccumulator::extractOldest() -> std::pair<KeyType, FrameVector> {
    auto oldest = partialEvents.extractOldest();
    finishedEvents.insert(oldest.first);
    return oldest;
}

auto FrameAccumulator::getFrameVector(KeyType key) -> FrameVector& {
    if (auto frvec = partialEvents.get(key)) {
        return *frvec;
    }
    else if (eventWasAlreadyFinished(key)) {
        throw EventAlreadyFinishedError(key);
    }
    else {
        return partialEvents.insert(key, FrameVector{});
    }
}

bool FrameAccumulator::eventWasAlreadyFinished(KeyType key) const {
    auto foundIter = finishedEvents.find(key);
    return foundIter != finishedEvents.end();
}

}
}
