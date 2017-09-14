#include "GRAWFrame.h"
#include <queue>
#include <cassert>

namespace attpc {
namespace mergers {

GRAWHeader::GRAWHeader(const std::vector<uint8_t>& rawFrame)
: metaType(rawFrame)
, frameSize(rawFrame)
, dataSource(rawFrame)
, frameType(rawFrame)
, revision(rawFrame)
, headerSize(rawFrame)
, itemSize(rawFrame)
, itemCount(rawFrame)
, eventTime(rawFrame)
, eventIdx(rawFrame)
, coboIdx(rawFrame)
, asadIdx(rawFrame)
, readOffset(rawFrame)
, status(rawFrame)
, hitPat_0(rawFrame)
, hitPat_1(rawFrame)
, hitPat_2(rawFrame)
, hitPat_3(rawFrame)
, multip_0(rawFrame)
, multip_1(rawFrame)
, multip_2(rawFrame)
, multip_3(rawFrame)
, windowOut(rawFrame)
{}

GRAWFrame::GRAWFrame(const std::vector<uint8_t>& rawFrame)
: header(rawFrame)
{
    const auto dataBegin = rawFrame.cbegin() + header.headerSize.value * 256;
    const auto dataEnd = dataBegin + header.itemSize.value * header.itemCount.value;
    assert(dataEnd <= rawFrame.cend());

    if (isFullReadout()) {
        data = decodeFullReadoutData(dataBegin, dataEnd);
    }
    else {
        data = decodePartialReadoutData(dataBegin, dataEnd);
    }
}

std::vector<GRAWDataItem> GRAWFrame::decodePartialReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                              const std::vector<uint8_t>::const_iterator& end) const {
    std::vector<GRAWDataItem> parsedData;

    for (auto iter = begin; iter != end; iter += 4) {
        const uint32_t encodedItem = utilities::parseValue<uint32_t>(iter, iter+4);

        GRAWDataItem item;
        item.aget       = (encodedItem & 0xC0000000) >> 30;
        item.channel    = (encodedItem & 0x3F800000) >> 23;
        item.timeBucket = (encodedItem & 0x007FC000) >> 14;
        item.sample     = (encodedItem & 0x00000FFF);

        parsedData.push_back(std::move(item));
    }

    return parsedData;
}

std::vector<GRAWDataItem> GRAWFrame::decodeFullReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                           const std::vector<uint8_t>::const_iterator& end) const {
    std::vector<std::queue<uint16_t>> sampleQueues (4);

    for (auto iter = begin; iter != end; iter += 2) {
        const uint16_t encodedItem = utilities::parseValue<uint16_t>(iter, iter + 2);
        const uint8_t  aget   = (encodedItem & 0xC000) >> 14;
        const uint16_t sample = (encodedItem & 0x0FFF);
        sampleQueues.at(aget).push(sample);
    }

    std::vector<GRAWDataItem> parsedData;

    for (uint8_t aget = 0; aget < 4; ++aget) {
        for (uint16_t timeBucket = 0; timeBucket < 512; ++timeBucket) {
            for (uint8_t channel = 0; channel < 68; ++channel) {
                uint16_t sample = sampleQueues.at(aget).front();
                sampleQueues.at(aget).pop();

                parsedData.emplace_back(aget, channel, timeBucket, sample);
            }
        }
    }

    return parsedData;
}

}
}
