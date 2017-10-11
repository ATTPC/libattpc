#ifndef ATTPC_MERGERS_MERGER
#define ATTPC_MERGERS_MERGER

#include <memory>
#include "attpc/mergers/FrameAccumulator.h"
#include "attpc/common/FullTraceEvent.h"
#include "attpc/common/PadLookupTable.h"

namespace attpc {
namespace mergers {

/**
 * @brief Builds and processes events from a collection of frames.
 *
 * While the grouping of frames is done in the FrameAccumulator, this class is responsible for transforming
 * the vectors of GRAWFrames extracted from the accumulator into valid FullTraceEvent objects. It also performs
 * a few transformations on the events to remove fixed-pattern noise, map the pad numbers, etc. Future transformations
 * could also be added to this class.
 */
class Merger {
public:
    /**
     * @brief Default constructor
     *
     * This builds a Merger without a pad map and with keepFPN set to false. Thus, pads will not be
     * mapped, and the fixed-pattern noise will be discarded.
     */
    Merger();
    /**
     * @brief Construct a Merger with a pad lookup table.
     *
     * In this case, the pads *will* be mapped unless `lookupPtr_` is `nullptr`.
     *
     * @param lookupPtr_ Pointer to the PadLookupTable.
     * @param keepFPN_   If true, do not discard the fixed-pattern noise channels.
     */
    Merger(std::shared_ptr<common::PadLookupTable> lookupPtr_, bool keepFPN_ = false);

    /**
     * @brief Merge the frames into an event and apply all transformations.
     *
     * After merging, fixed-pattern noise will be discarded (depending on the value of keepFPN),
     * and pads will be mapped (if the lookupPtr is not `nullptr`).
     *
     * @param  frames The collection of frames to merge.
     * @return        The built and processed event.
     */
    attpc::common::FullTraceEvent mergeAndProcessEvent(const FrameAccumulator::FrameVector& frames) const;

    /**
     * @brief Merge the given collection of frames into an event.
     *
     * No transformations are performed after building the event.
     *
     * @param  frames The frames to merge.
     * @return        The built event.
     */
    attpc::common::FullTraceEvent mergeFrames(const FrameAccumulator::FrameVector& frames) const;

    /**
     * @brief Discard the fixed-pattern noise channels (11, 22, 45, 56) in the event.
     * @param event The event. It will be modified in-place.
     */
    void discardFPN(common::FullTraceEvent& event) const;

    /**
     * @brief Map the pad numbers in the event using the PadLookupTable.
     *
     * If the pointer to the lookup table is `nullptr`, this will have no effect. It is still safe to call,
     * however, and it will not try to dereference a null pointer.
     *
     * @param event The event to map. It will be modified in-place.
     */
    void setPadNumbers(common::FullTraceEvent& event) const;

private:
    //! Pointer to the pad lookup table
    std::shared_ptr<common::PadLookupTable> lookupPtr;
    //! The fixed-pattern noise channel numbers
    static const std::vector<channelid_type> fpnChannels;
    //! If true, do not discard the fixed-pattern noise channels
    bool keepFPN;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_MERGER */
