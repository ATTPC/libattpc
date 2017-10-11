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
 * This class does the actual "merging," in some sense. It maintains a cache that contains a vector of frames for
 * each event. Frames can be added to this cache using addFrame. Then, to actually build the event, call extractOldest
 * to pull the least recently used frame vector out of the cache, and build the event from these frames.
 *
 * The accumulator maintains a list of each event that is removed via extractOldest, and it will issue an error
 * if a frame for a previously extracted event is later added. This way, if a frame from an old event is randomly
 * mixed into a later part of the GRAW file, it won't get dropped silently.
 */
class FrameAccumulator {
public:
    //! The type of the key used to group frames by event. This is either the event ID or the timestamp.
    using KeyType = uint64_t;
    //! A vector of GRAWFrame instances.
    using FrameVector = std::vector<GRAWFrame>;

    /**
     * @brief Construct a new instance
     *
     * This takes one argument which specifies the merging method. Use attpc::mergers::GetMergeKeyFromEventId
     * to merge on event ID, and use attpc::mergers::GetMergeKeyFromTimestamp to merge by timestamp.
     *
     * @param method The merging method. See above.
     */
    FrameAccumulator(MergeKeyFunction method) : getMergeKey(method) {}

    /**
     * @brief Add a frame to the accumulator.
     *
     * Frames will be added to the appropriate FrameVector based on the merging key extracted from them.
     * (That is, they will be grouped by event ID or by timestamp depending on what merging method is
     * being used.)
     *
     * @param  frame The frame to add.
     *
     * @throws EventAlreadyFinishedError If the frame is from an event that was previously extracted from
     *                                   the FrameAccumulator.
     */
    void addFrame(const GRAWFrame& frame);
    //! @overload
    void addFrame(GRAWFrame&& frame);

    /**
     * @brief Extract the least recently modified FrameVector from the accumulator.
     *
     * When a frame is added to the accumulator, the corresponding FrameVector is moved to the front of
     * the partial event cache. This method extracts the FrameVector at the back of that cache, which is
     * the vector that had frames added to it least recently. This event is most likely to be complete if
     * the frames are roughly in order in the GRAW files.
     *
     * @return A pair containing the key and the FrameVector from the least-recent event.
     */
    std::pair<KeyType, FrameVector> extractOldest();

    //! Get the number of events in the accumulator.
    size_t size() const { return partialEvents.size(); }

    //! @brief Indicates that the frame being added corresponds to an event that was previously extracted from this FrameAccumulator.
    class EventAlreadyFinishedError : public std::runtime_error {
    public:
        /**
         * @brief Construct an instance of this exception.
         * @param key The event merge key (i.e. the event ID or the timestamp)
         */
        EventAlreadyFinishedError(KeyType key)
            : std::runtime_error("Event " + std::to_string(key) + " was already finished.") {}
    };

private:
    /**
     * @brief Get the FrameVector that corresponds to this key.
     *
     * A new FrameVector is created if this event key has not been seen before.
     *
     * @param  key The event merge key (event ID or timestamp)
     * @return     The FrameVector for this event.
     *
     * @throws EventAlreadyFinishedError If this event was previously extracted from this FrameAccumulator.
     */
    FrameVector& getFrameVector(KeyType key);

    //! True if this event was previously extracted from this FrameAccumulator.
    bool eventWasAlreadyFinished(KeyType key) const;

    MergeKeyFunction getMergeKey;  //! The function that extracts a merge key from each event
    LRUQueue<KeyType, FrameVector> partialEvents;  //! Structure storing the FrameVectors
    std::set<KeyType> finishedEvents;  //! A listing of each event merge key that has been extracted
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_FRAMEACCUMULATOR_H */
