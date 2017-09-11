#ifndef ATTPC_MERGERS_GRAWFILE_H
#define ATTPC_MERGERS_GRAWFILE_H

#include <fstream>
#include <string>
#include "GRAWFrame.h"
#include "GRAWFormatSpec.h"
#include <vector>
#include <tuple>

namespace attpc {
namespace mergers {

class GRAWFile {
public:
    GRAWFile(const std::string& filename, const bool readonly = true);

    void open(const std::string& filename, const bool readonly = true);
    void close();

    GRAWFrame readFrame();

    GRAWHeader decodeHeader(const std::vector<uint8_t>& rawFrame) const;
    GRAWDataItem decodePartialReadoutDataItem(const uint32_t encodedItem) const;
    std::tuple<uint8_t, uint16_t> decodeFullReadoutDataItem(const uint16_t encodedItem) const;
    std::vector<GRAWDataItem> extractPartialReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                        const std::vector<uint8_t>::const_iterator& end) const;
    std::vector<GRAWDataItem> extractFullReadoutData(const std::vector<uint8_t>::const_iterator& begin,
                                                     const std::vector<uint8_t>::const_iterator& end) const;

    GRAWFrame decodeFrame(const std::vector<uint8_t>& rawFrame);
    void encodeFrame(const GRAWFrame& frame);

private:
    std::fstream file;
    HeaderFormat spec;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFILE_H */
