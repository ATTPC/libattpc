#include "catch.hpp"
#include "attpc/mergers/FrameAccumulator.h"
#include <random>
#include <algorithm>
#include <iterator>
#include <functional>

using namespace attpc::mergers;

namespace {

using KeySetter = std::function<void(GRAWFrame&, uint64_t)>;
void setEventId(GRAWFrame& frame, uint64_t eventId) { frame.getEventId() = eventId; }
void setTimestamp(GRAWFrame& frame, uint64_t timestamp) { frame.getTimestamp() = timestamp; }

FrameAccumulator::FrameVector makeTestFrameVector(KeySetter setKey, uint64_t key, int numFrames) {
    FrameAccumulator::FrameVector fvec;
    for (int i = 0; i < numFrames; ++i) {
        GRAWFrame frame;
        // Set both keys to varying values first. That way we can be sure
        // that the correct key was merged on when we test later since the *other* key
        // will not contain the same value for each frame.
        frame.getEventId() = i;
        frame.getTimestamp() = i*2;
        setKey(frame, key);
        fvec.push_back(std::move(frame));
    }
    return fvec;
}

void testAccumulation(KeySetter setkey, MergeKeyFunction getkey) {
    FrameAccumulator::FrameVector evt0Frames = makeTestFrameVector(setkey, 0, 4);
    FrameAccumulator::FrameVector evt1Frames = makeTestFrameVector(setkey, 1, 5);

    FrameAccumulator accum {getkey};

    SECTION("Frames with same key go in same event") {
        for (const auto& frame : evt0Frames) {
            accum.addFrame(frame);
            REQUIRE(accum.size() == 1);
        }
        for (const auto& frame : evt1Frames) {
            accum.addFrame(frame);
            REQUIRE(accum.size() == 2);
        }
        while (accum.size() > 0) {
            FrameAccumulator::KeyType key;
            FrameAccumulator::FrameVector fvec;
            std::tie(key, fvec) = accum.extractOldest();
            bool allHaveRightEventIdx = std::all_of(fvec.begin(), fvec.end(), [key, getkey](const GRAWFrame& f){
                return getkey(f) == key;
            });
            REQUIRE(allHaveRightEventIdx);
        }
    }
}

}

TEST_CASE("Can accumulate frames by event ID", "[FrameAccumulator]") {
    testAccumulation(setEventId, GetMergeKeyFromEventId);
}

TEST_CASE("Can accumulate frames by timestamp", "[FrameAccumulator]") {
    testAccumulation(setTimestamp, GetMergeKeyFromTimestamp);
}

TEST_CASE("Can extract oldest FrameVector from FrameAccumulator", "[FrameAccumulator]") {
    FrameAccumulator::FrameVector frames;
    for (unsigned evtId = 0; evtId < 4; ++evtId) {
        auto v = makeTestFrameVector(setEventId, evtId, 4);
        for (const auto& frame : v) {
            frames.push_back(frame);
        }
    }
    std::mt19937 rng {0};
    std::shuffle(frames.begin(), frames.end(), rng);

    FrameAccumulator accum {GetMergeKeyFromEventId};
    for (const auto& frame : frames) {
        accum.addFrame(frame);
    }

    // Ordering of events in `accum` is based on the ordering of frames in `frames`. The most
    // *recent* event in `accum` corresponds to the *last* frame in `frames`. So, extract all of
    // them and look at them in the reverse order.
    std::vector<decltype(accum.extractOldest())> eventPairs;
    while (accum.size() > 0) {
        eventPairs.push_back(accum.extractOldest());
    }

    // Reverse orders so *newest* events are first
    std::reverse(eventPairs.begin(), eventPairs.end());
    std::reverse(frames.begin(), frames.end());

    for (const auto& eventPair : eventPairs) {
        FrameAccumulator::KeyType evtId;
        FrameAccumulator::FrameVector fvec;
        std::tie(evtId, fvec) = eventPair;

        REQUIRE(frames.front().getEventId() == evtId);  // Asserts that the order is correct

        // Remove frames from this event from `frames`
        auto thisEventPredicate = [evtId](const GRAWFrame& f) { return f.getEventId() == evtId; };
        auto newEnd = std::remove_if(frames.begin(), frames.end(), thisEventPredicate);
        frames.erase(newEnd, frames.end());
    }
}

TEST_CASE("FrameAccumulator keeps track of finished events", "[FrameAccumulator]") {
    FrameAccumulator accum {GetMergeKeyFromEventId};
    for (unsigned evtId = 0; evtId < 4; ++evtId) {
        GRAWFrame frame;
        frame.getEventId() = evtId;
        accum.addFrame(frame);
    }

    auto eventPair = accum.extractOldest();
    auto& removedFrameVector = eventPair.second;

    REQUIRE_THROWS_AS(accum.addFrame(removedFrameVector.front()), FrameAccumulator::EventAlreadyFinishedError);
}
