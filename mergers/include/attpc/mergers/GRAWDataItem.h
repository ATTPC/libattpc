#ifndef ATTPC_MERGERS_GRAWDATAITEM_H
#define ATTPC_MERGERS_GRAWDATAITEM_H

#include <cstdint>

namespace attpc {
namespace mergers {

struct GRAWDataItem {
    GRAWDataItem() = default;
    GRAWDataItem(uint8_t aget_, uint8_t channel_, uint16_t timeBucket_, uint16_t sample_)
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

}
}

#endif /* end of include guard: ATTPC_MERGERS_GRAWDATAITEM_H */
