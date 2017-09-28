#ifndef ATTPC_MERGERS_GRAWHEADER_H
#define ATTPC_MERGERS_GRAWHEADER_H

#include <cstdint>
#include <bitset>
#include "attpc/mergers/GRAWHeaderField.h"

namespace attpc {
namespace mergers {

class GRAWHeader {
public:
    GRAWHeader() = default;
    GRAWHeader(const RawFrame& rawFrame);

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

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWHEADER_H */
