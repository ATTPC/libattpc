#ifndef ATTPC_MERGERS_GRAWFRAME_H
#define ATTPC_MERGERS_GRAWFRAME_H

#include <cstdint>
#include <vector>
#include "attpc/mergers/GRAWHeader.h"
#include "attpc/mergers/GRAWDataItem.h"

namespace attpc {
namespace mergers {

/**
 * @brief Represents a single frame from a GRAWFile
 *
 * Frames contain a header and a set of data items. The representation used here is intented to map directly onto
 * the specification of the frame structure given by the GET collaboration. Thus, most of the header properties
 * should be read directly from the header object.
 */
class GRAWFrame {
public:
    //! Construct an empty GRAWFrame
    GRAWFrame() = default;
    //! Construct a GRAWFrame by parsing the data in the given RawFrame.
    GRAWFrame(const RawFrame& rawFrame);

    //! True if this frame was recorded in full-readout mode
    bool isFullReadout() const { return header.frameType.value == fullReadoutFrameType; }

    //! Returns the event ID from the header
    auto getEventId()       -> decltype(GRAWHeader::eventIdx)::type& { return header.eventIdx.value; }
    //! @overload
    auto getEventId() const -> decltype(GRAWHeader::eventIdx)::type  { return header.eventIdx.value; }

    //! Returns the event timestamp from the header
    auto getTimestamp()       -> decltype(GRAWHeader::eventTime)::type& { return header.eventTime.value; }
    //! @overload
    auto getTimestamp() const -> decltype(GRAWHeader::eventTime)::type  { return header.eventTime.value; }


public:
    //! The header, which contains information like the CoBo and AsAd indices, timestamp, etc.
    GRAWHeader header;
    //! The set of data items present in this frame.
    std::vector<GRAWDataItem> data;

private:
    std::vector<GRAWDataItem> decodePartialReadoutData(const RawFrame::const_iterator& begin,
                                                       const RawFrame::const_iterator& end) const;
    std::vector<GRAWDataItem> decodeFullReadoutData(const RawFrame::const_iterator& begin,
                                                    const RawFrame::const_iterator& end) const;

    static const uint8_t fullReadoutFrameType = 2;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFRAME_H */
