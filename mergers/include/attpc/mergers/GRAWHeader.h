#ifndef ATTPC_MERGERS_GRAWHEADER_H
#define ATTPC_MERGERS_GRAWHEADER_H

#include <cstdint>
#include <bitset>
#include "attpc/mergers/GRAWHeaderField.h"

namespace attpc {
namespace mergers {

/**
 * @brief Represents the header of a single GRAWFrame
 *
 * The fields in this header are directly taken from the description of the CoBo data format. More details
 * about each field can be found in that document.
 *
 * Fields are stored using GRAWHeaderField instances. A consequence of this is that each field is technically
 * a different type due to the different template parameters. To access the value of each field, use the
 * attribute GRAWHeaderField::value.
 */
class GRAWHeader {
public:
    //! Construct an empty header
    GRAWHeader() = default;
    //! Construct a header by parsing the given raw frame
    GRAWHeader(const RawFrame& rawFrame);

    //! The frame type. This should always be 8 for version 3.3 of the data format.
    GRAWHeaderField<uint8_t, 0> metaType;
    //! The size of the frame in units of 256 bytes
    GRAWHeaderField<uint32_t, 1, 3> frameSize;
    //! The data source ID as set by the CoBo
    GRAWHeaderField<uint8_t, 4> dataSource;
    //! The type of the frame. Set to 1 for partial readout, and 2 for full readout.
    GRAWHeaderField<uint16_t, 5, 2> frameType;
    //! The revision number of the format specification. Should be 5.
    GRAWHeaderField<uint8_t, 7> revision;
    //! The size of the header in units of 256 bytes.
    GRAWHeaderField<uint16_t, 8, 2> headerSize;
    //! The size of a data item, in bytes.
    GRAWHeaderField<uint16_t, 10, 2> itemSize;
    //! The number of data items in the frame.
    GRAWHeaderField<uint32_t, 12, 4> itemCount;
    //! The timestamp of the event
    GRAWHeaderField<uint64_t, 16, 6> eventTime;
    //! The event ID
    GRAWHeaderField<uint32_t, 22, 4> eventIdx;
    //! The CoBo ID
    GRAWHeaderField<uint8_t, 26> coboIdx;
    //! The AsAd ID
    GRAWHeaderField<uint8_t, 27> asadIdx;
    //! The AGET readout offset. See documentation for details.
    GRAWHeaderField<uint16_t, 28, 2> readOffset;
    //! The frame status (undefined currently).
    GRAWHeaderField<uint8_t, 30> status;
    //! AGET 0 hit pattern
    GRAWHeaderField<std::bitset<9*8>, 31, 9> hitPat_0;
    //! AGET 1 hit pattern
    GRAWHeaderField<std::bitset<9*8>, 40, 9> hitPat_1;
    //! AGET 2 hit pattern
    GRAWHeaderField<std::bitset<9*8>, 49, 9> hitPat_2;
    //! AGET 3 hit pattern
    GRAWHeaderField<std::bitset<9*8>, 58, 9> hitPat_3;
    //! AGET 0 multiplicity
    GRAWHeaderField<uint16_t, 67, 2> multip_0;
    //! AGET 1 multiplicity
    GRAWHeaderField<uint16_t, 69, 2> multip_1;
    //! AGET 2 multiplicity
    GRAWHeaderField<uint16_t, 71, 2> multip_2;
    //! AGET 3 multiplicity
    GRAWHeaderField<uint16_t, 73, 2> multip_3;
    //! Value of the sliding multiplicity window at the time of the trigger.
    GRAWHeaderField<uint32_t, 75, 4> windowOut;
    //! Index of the last cell read by AGET 0
    GRAWHeaderField<uint16_t, 79, 2> lastCell_0;
    //! Index of the last cell read by AGET 1
    GRAWHeaderField<uint16_t, 81, 2> lastCell_1;
    //! Index of the last cell read by AGET 2
    GRAWHeaderField<uint16_t, 83, 2> lastCell_2;
    //! Index of the last cell read by AGET 3
    GRAWHeaderField<uint16_t, 85, 2> lastCell_3;
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWHEADER_H */
