#include "GRAWFile.h"
#include <cassert>
#include <type_traits>
#include <stdexcept>
#include <queue>

namespace {

template <class T, class Iterator>
T parseValue(const Iterator& begin, const Iterator& end) {
    static_assert(!(std::is_integral<T>::value && std::is_signed<T>::value),
                  "Left bit shifts on signed integers can invoke undefined behavior, so this is not allowed.");

    const size_t numBytes = std::distance(begin, end);
    assert(numBytes <= sizeof(T));

    T result = 0;
    for (auto iter = end - 1; iter >= begin; --iter) {
        const size_t offset = std::distance(begin, iter);
        T byte = *iter;
        result |= (byte << 8u*offset);
    }
    return result;
}

template <class T>
T getHeaderField(const std::vector<uint8_t>& rawFrame, const attpc::mergers::HeaderFormatField& specField) {
    const auto fieldBegin = rawFrame.begin() + specField.offset;
    const auto fieldEnd = fieldBegin + specField.size;
    if (fieldEnd > rawFrame.end()) {
        throw std::runtime_error("Header field was past the end of the frame.");
    }
    return parseValue<T>(fieldBegin, fieldEnd);
}

}

namespace attpc {
namespace mergers {

GRAWFile::GRAWFile(const std::string& filename, const bool readonly) {
    open(filename, readonly);
}

void GRAWFile::open(const std::string& filename, const bool readonly) {
    std::ios_base::openmode flags;
    if (readonly) {
        flags = std::ios_base::binary | std::ios_base::in;
    }
    else {
        flags = std::ios_base::binary | std::ios_base::in | std::ios_base::out;
    }

    file = std::fstream(filename, flags);
}

void GRAWFile::close() {
    file.close();
}

GRAWFrame GRAWFile::readFrame() {
    const auto start = file.tellg();
    file.seekg(spec.frameSize.offset, std::ios_base::cur);
    std::vector<uint8_t> rawSize (spec.frameSize.size);
    file.read(reinterpret_cast<char*>(rawSize.data()), rawSize.size());
    uint32_t frameSize = parseValue<uint32_t>(rawSize.begin(), rawSize.end());
    file.seekg(start);

    std::vector<uint8_t> rawFrame (frameSize * 256);
    file.read(reinterpret_cast<char*>(rawFrame.data()), rawFrame.size());

    return decodeFrame(rawFrame);
}

GRAWHeader GRAWFile::decodeHeader(const std::vector<uint8_t>& rawFrame) const {
    GRAWHeader header;

    header.metaType = getHeaderField<decltype(header.metaType)>(rawFrame, spec.metaType);
    header.frameSize = getHeaderField<decltype(header.frameSize)>(rawFrame, spec.frameSize);
    header.dataSource = getHeaderField<decltype(header.dataSource)>(rawFrame, spec.dataSource);
    header.frameType = getHeaderField<decltype(header.frameType)>(rawFrame, spec.frameType);
    header.revision = getHeaderField<decltype(header.revision)>(rawFrame, spec.revision);
    header.headerSize = getHeaderField<decltype(header.headerSize)>(rawFrame, spec.headerSize);
    header.itemSize = getHeaderField<decltype(header.itemSize)>(rawFrame, spec.itemSize);
    header.itemCount = getHeaderField<decltype(header.itemCount)>(rawFrame, spec.itemCount);
    header.eventTime = getHeaderField<decltype(header.eventTime)>(rawFrame, spec.eventTime);
    header.eventIdx = getHeaderField<decltype(header.eventIdx)>(rawFrame, spec.eventIdx);
    header.coboIdx = getHeaderField<decltype(header.coboIdx)>(rawFrame, spec.coboIdx);
    header.asadIdx = getHeaderField<decltype(header.asadIdx)>(rawFrame, spec.asadIdx);
    header.readOffset = getHeaderField<decltype(header.readOffset)>(rawFrame, spec.readOffset);
    header.status = getHeaderField<decltype(header.status)>(rawFrame, spec.status);
    header.hitPat_0 = getHeaderField<decltype(header.hitPat_0)>(rawFrame, spec.hitPat_0);
    header.hitPat_1 = getHeaderField<decltype(header.hitPat_1)>(rawFrame, spec.hitPat_1);
    header.hitPat_2 = getHeaderField<decltype(header.hitPat_2)>(rawFrame, spec.hitPat_2);
    header.hitPat_3 = getHeaderField<decltype(header.hitPat_3)>(rawFrame, spec.hitPat_3);
    header.multip_0 = getHeaderField<decltype(header.multip_0)>(rawFrame, spec.multip_0);
    header.multip_1 = getHeaderField<decltype(header.multip_1)>(rawFrame, spec.multip_1);
    header.multip_2 = getHeaderField<decltype(header.multip_2)>(rawFrame, spec.multip_2);
    header.multip_3 = getHeaderField<decltype(header.multip_3)>(rawFrame, spec.multip_3);
    header.windowOut = getHeaderField<decltype(header.windowOut)>(rawFrame, spec.windowOut);

    return header;
}

GRAWDataItem GRAWFile::decodePartialReadoutDataItem(const uint32_t encodedItem) const {
    GRAWDataItem item;
    item.aget       = (encodedItem & 0xC0000000) >> 30;
    item.channel    = (encodedItem & 0x3F800000) >> 23;
    item.timeBucket = (encodedItem & 0x007FC000) >> 14;
    item.sample     = (encodedItem & 0x00000FFF);

    return item;
}

std::tuple<uint8_t, uint16_t> GRAWFile::decodeFullReadoutDataItem(const uint16_t encodedItem) const {
    uint8_t  aget   = (encodedItem & 0xC000) >> 14;
    uint16_t sample = (encodedItem & 0x0FFF);

    return {aget, sample};
}

std::vector<GRAWDataItem> GRAWFile::extractPartialReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                              const std::vector<uint8_t>::const_iterator& end) const {
    std::vector<GRAWDataItem> data;

    for (auto iter = begin; iter != end; iter += 4) {
        const uint32_t encodedItem = parseValue<uint32_t>(iter, iter+4);
        data.push_back(decodePartialReadoutDataItem(encodedItem));
    }

    return data;
}

std::vector<GRAWDataItem> GRAWFile::extractFullReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                           const std::vector<uint8_t>::const_iterator& end) const {
    std::vector<std::queue<uint16_t>> sampleQueues (4);

    for (auto iter = begin; iter != end; iter += 2) {
        const uint16_t encodedItem = parseValue<uint16_t>(iter, iter + 2);
        uint8_t aget;
        uint16_t sample;
        std::tie(aget, sample) = decodeFullReadoutDataItem(encodedItem);
        sampleQueues.at(aget).push(sample);
    }

    std::vector<GRAWDataItem> data;

    for (uint8_t aget = 0; aget < 4; ++aget) {
        for (uint16_t timeBucket = 0; timeBucket < 512; ++timeBucket) {
            for (uint8_t channel = 0; channel < 68; ++channel) {
                uint16_t sample = sampleQueues.at(aget).front();
                sampleQueues.at(aget).pop();

                data.emplace_back(aget, channel, timeBucket, sample);
            }
        }
    }

    return data;
}

GRAWFrame GRAWFile::decodeFrame(const std::vector<uint8_t>& rawFrame) {
    GRAWFrame frame;

    frame.header = decodeHeader(rawFrame);

    const auto dataBegin = rawFrame.cbegin() + frame.header.headerSize * 256;
    const auto dataEnd = dataBegin + frame.header.itemSize * frame.header.itemCount;
    assert(dataEnd <= rawFrame.cend());

    if (frame.isFullReadout()) {
        frame.data = extractFullReadoutData(dataBegin, dataEnd);
    }
    else {
        frame.data = extractPartialReadoutData(dataBegin, dataEnd);
    }

    return frame;
}

}
}
