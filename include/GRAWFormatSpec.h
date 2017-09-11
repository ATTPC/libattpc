#ifndef ATTPC_MERGERS_GRAWFORMATSPEC_H
#define ATTPC_MERGERS_GRAWFORMATSPEC_H

#include <cstdint>

namespace attpc {
namespace mergers {

struct HeaderFormatField {
    int offset;
    int size = 1;
};

struct HeaderFormat {
    HeaderFormatField metaType {0};
    HeaderFormatField frameSize {1, 3};
    HeaderFormatField dataSource {4};
    HeaderFormatField frameType {5, 2};
    HeaderFormatField revision {7};
    HeaderFormatField headerSize {8, 2};
    HeaderFormatField itemSize {10, 2};
    HeaderFormatField itemCount {12, 4};
    HeaderFormatField eventTime {16, 6};
    HeaderFormatField eventIdx {22, 4};
    HeaderFormatField coboIdx {26};
    HeaderFormatField asadIdx {27};
    HeaderFormatField readOffset {28, 2};
    HeaderFormatField status {30};
    HeaderFormatField hitPat_0 {31, 9};
    HeaderFormatField hitPat_1 {40, 9};
    HeaderFormatField hitPat_2 {49, 9};
    HeaderFormatField hitPat_3 {58, 9};
    HeaderFormatField multip_0 {67, 2};
    HeaderFormatField multip_1 {69, 2};
    HeaderFormatField multip_2 {71, 2};
    HeaderFormatField multip_3 {73, 2};
    HeaderFormatField windowOut {75, 4};
    HeaderFormatField lastCell_0 {79, 2};
    HeaderFormatField lastCell_1 {81, 2};
    HeaderFormatField lastCell_2 {83, 2};
    HeaderFormatField lastCell_3 {85, 2};
};

namespace SpecConstants {
    const uint8_t partialReadoutFrameType = 1;
    const uint8_t fullReadoutFrameType = 2;
}

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWFORMATSPEC_H */
