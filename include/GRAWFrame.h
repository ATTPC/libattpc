#ifndef ATTPC_MERGERS_GRAWFRAME_H
#define ATTPC_MERGERS_GRAWFRAME_H

#include <cstdint>
#include <bitset>
#include <vector>
#include "utilities.h"

namespace attpc {
namespace mergers {

template <class T, size_t offset_, size_t size_ = 1>
struct GRAWHeaderField {
    static_assert(sizeof(T) >= size_, "Declared type is too small to hold a value with the given size");

    GRAWHeaderField() = default;
    GRAWHeaderField(const std::vector<uint8_t>& rawFrame) : value(extractFromRawFrame(rawFrame)) {}

    using type = T;
    static constexpr size_t offset = offset_;
    static constexpr size_t size = size_;
    type value;

    static type extractFromRawFrame(const std::vector<uint8_t>& rawFrame) {
        const auto fieldBegin = rawFrame.begin() + offset;
        const auto fieldEnd = fieldBegin + size;
        if (fieldEnd > rawFrame.end()) {
            throw std::runtime_error("Header field was past the end of the frame.");
        }
        return utilities::parseValue<type>(fieldBegin, fieldEnd);
    }
};

class GRAWHeader {
public:
    GRAWHeader() = default;
    GRAWHeader(const std::vector<uint8_t>& rawFrame);

public:
    GRAWHeaderField<uint8_t, 0> metaType;
    GRAWHeaderField<uint32_t, 1, 3> frameSize;
    GRAWHeaderField<uint8_t, 4> dataSource;
    GRAWHeaderField<uint16_t, 5, 2> frameType;
    GRAWHeaderField<uint8_t, 7> revision;
    GRAWHeaderField<uint16_t, 8, 2> headerSize;
    GRAWHeaderField<uint16_t, 10, 2> itemSize;
    GRAWHeaderField<uint32_t, 12, 4> itemCount;
    GRAWHeaderField<uint64_t, 16, 6> eventTime;
    GRAWHeaderField<uint32_t, 22, 4> eventIdx;
    GRAWHeaderField<uint8_t, 26> coboIdx;
    GRAWHeaderField<uint8_t, 27> asadIdx;
    GRAWHeaderField<uint16_t, 28, 2> readOffset;
    GRAWHeaderField<uint8_t, 30> status;
    GRAWHeaderField<std::bitset<9*8>, 31, 9> hitPat_0;
    GRAWHeaderField<std::bitset<9*8>, 40, 9> hitPat_1;
    GRAWHeaderField<std::bitset<9*8>, 49, 9> hitPat_2;
    GRAWHeaderField<std::bitset<9*8>, 58, 9> hitPat_3;
    GRAWHeaderField<uint16_t, 67, 2> multip_0;
    GRAWHeaderField<uint16_t, 69, 2> multip_1;
    GRAWHeaderField<uint16_t, 71, 2> multip_2;
    GRAWHeaderField<uint16_t, 73, 2> multip_3;
    GRAWHeaderField<uint32_t, 75, 4> windowOut;
    GRAWHeaderField<uint16_t, 79, 2> lastCell_0;
    GRAWHeaderField<uint16_t, 81, 2> lastCell_1;
    GRAWHeaderField<uint16_t, 83, 2> lastCell_2;
    GRAWHeaderField<uint16_t, 85, 2> lastCell_3;
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

class GRAWFrame {
public:
    GRAWFrame() = default;
    GRAWFrame(const std::vector<uint8_t>& rawFrame);

    bool isFullReadout() const { return header.frameType.value == fullReadoutFrameType; }


public:
    GRAWHeader header;
    std::vector<GRAWDataItem> data;

private:
    std::vector<GRAWDataItem> decodePartialReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                       const std::vector<uint8_t>::const_iterator& end) const;
    std::vector<GRAWDataItem> decodeFullReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                    const std::vector<uint8_t>::const_iterator& end) const;

    static const uint8_t fullReadoutFrameType = 2;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFRAME_H */
