#ifndef ATTPC_MERGERS_GRAWFRAME_H
#define ATTPC_MERGERS_GRAWFRAME_H

#include <cstdint>
#include <bitset>
#include <vector>
#include "GRAWFormatSpec.h"

namespace attpc {
namespace mergers {

template <class T>
struct GRAWHeaderField {
    using type = T;

    explicit GRAWHeaderField(const size_t offset_)
    : offset(offset_)
    , size(1)
    , value(0)
    {}
    GRAWHeaderField(const size_t offset_, const size_t size_)
    : offset(offset_)
    , size(size_)
    , value(0)
    {}
    GRAWHeaderField(const size_t offset_, const size_t size_, const type value_)
    : offset(offset_)
    , size(size_)
    , value(value_)
    {}

    size_t offset;
    size_t size;
    type value;
};

struct GRAWHeader {
    GRAWHeaderField<uint8_t> metaType {0};
    GRAWHeaderField<uint32_t> frameSize {1, 3};
    GRAWHeaderField<uint8_t> dataSource {4};
    GRAWHeaderField<uint16_t> frameType {5, 2};
    GRAWHeaderField<uint8_t> revision {7};
    GRAWHeaderField<uint16_t> headerSize {8, 2};
    GRAWHeaderField<uint16_t> itemSize {10, 2};
    GRAWHeaderField<uint32_t> itemCount {12, 4};
    GRAWHeaderField<uint64_t> eventTime {16, 6};
    GRAWHeaderField<uint32_t> eventIdx {22, 4};
    GRAWHeaderField<uint8_t> coboIdx {26};
    GRAWHeaderField<uint8_t> asadIdx {27};
    GRAWHeaderField<uint16_t> readOffset {28, 2};
    GRAWHeaderField<uint8_t> status {30};
    GRAWHeaderField<std::bitset<9*8>> hitPat_0 {31, 9};
    GRAWHeaderField<std::bitset<9*8>> hitPat_1 {40, 9};
    GRAWHeaderField<std::bitset<9*8>> hitPat_2 {49, 9};
    GRAWHeaderField<std::bitset<9*8>> hitPat_3 {58, 9};
    GRAWHeaderField<uint16_t> multip_0 {67, 2};
    GRAWHeaderField<uint16_t> multip_1 {69, 2};
    GRAWHeaderField<uint16_t> multip_2 {71, 2};
    GRAWHeaderField<uint16_t> multip_3 {73, 2};
    GRAWHeaderField<uint32_t> windowOut {75, 4};
    GRAWHeaderField<uint16_t> lastCell_0 {79, 2};
    GRAWHeaderField<uint16_t> lastCell_1 {81, 2};
    GRAWHeaderField<uint16_t> lastCell_2 {83, 2};
    GRAWHeaderField<uint16_t> lastCell_3 {85, 2};
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

    bool isFullReadout() const { return header.frameType.value == SpecConstants::fullReadoutFrameType; }
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFRAME_H */
