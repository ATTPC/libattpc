#ifndef ATTPC_MERGERS_GRAWFRAME_H
#define ATTPC_MERGERS_GRAWFRAME_H

#include <cstdint>
#include <vector>
#include "attpc/mergers/GRAWHeader.h"
#include "attpc/mergers/GRAWDataItem.h"

namespace attpc {
namespace mergers {

class GRAWFrame {
public:
    GRAWFrame() = default;
    GRAWFrame(const RawFrame& rawFrame);

    bool isFullReadout() const { return header.frameType.value == fullReadoutFrameType; }


public:
    GRAWHeader header;
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
