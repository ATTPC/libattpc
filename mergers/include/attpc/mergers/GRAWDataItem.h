#ifndef ATTPC_MERGERS_GRAWDATAITEM_H
#define ATTPC_MERGERS_GRAWDATAITEM_H

#include <cstdint>

namespace attpc {
namespace mergers {

/**
 * @brief Represents a single data item (sample) from a GRAWFrame.
 *
 * This most closely models a data item from partial readout mode, but this class is also used when
 * unpacking full-readout-mode data items for consistency.
 */
struct GRAWDataItem {
    //! @brief Default constructor.
    GRAWDataItem() = default;
    /**
     * @brief Construct a data item with the given values
     * @param aget_       The AGET index
     * @param channel_    The channel number
     * @param timeBucket_ The time bucket index
     * @param sample_     The sample value
     */
    GRAWDataItem(uint8_t aget_, uint8_t channel_, uint16_t timeBucket_, uint16_t sample_)
    : aget(aget_)
    , channel(channel_)
    , timeBucket(timeBucket_)
    , sample(sample_)
    {}

    uint8_t aget;          //! The AGET index
    uint8_t channel;       //! The channel number
    uint16_t timeBucket;   //! The time bucket index
    uint16_t sample;       //! The sample value
};

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWDATAITEM_H */
