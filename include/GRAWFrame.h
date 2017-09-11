#ifndef ATTPC_MERGERS_GRAWFRAME_H
#define ATTPC_MERGERS_GRAWFRAME_H

#include <cstdint>
#include <bitset>
#include <vector>
#include "GRAWFormatSpec.h"

namespace attpc {
namespace mergers {

struct GRAWHeader {
    uint8_t metaType;
    uint32_t frameSize;
    uint8_t dataSource;
    uint16_t frameType;
    uint8_t revision;
    uint16_t headerSize;
    uint16_t itemSize;
    uint32_t itemCount;
    uint64_t eventTime;
    uint32_t eventIdx;
    uint8_t coboIdx;
    uint8_t asadIdx;
    uint16_t readOffset;
    uint8_t status;
    std::bitset<9*8> hitPat_0;
    std::bitset<9*8> hitPat_1;
    std::bitset<9*8> hitPat_2;
    std::bitset<9*8> hitPat_3;
    uint16_t multip_0;
    uint16_t multip_1;
    uint16_t multip_2;
    uint16_t multip_3;
    uint32_t windowOut;
    uint16_t lastCell_0;
    uint16_t lastCell_1;
    uint16_t lastCell_2;
    uint16_t lastCell_3;
};

struct GRAWDataItem {
    GRAWDataItem() = default;
    GRAWDataItem(const uint8_t aget_, const uint8_t channel_,
                 const uint16_t timeBucket_, const uint16_t sample_)
    : aget(aget_)
    , channel(channel_)
    , timeBucket(timeBucket_)
    , sample(sample_)
    {}

    uint8_t aget;
    uint8_t channel;
    uint16_t timeBucket;
    uint16_t sample;
};

struct GRAWFrame {
    GRAWHeader header;
    std::vector<GRAWDataItem> data;

    bool isFullReadout() const { return header.frameType == SpecConstants::fullReadoutFrameType; }
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFRAME_H */
