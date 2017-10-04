#ifndef ATTPC_COMMON_TYPES_H
#define ATTPC_COMMON_TYPES_H

#include <cstdint>

namespace attpc {

using coboid_type = int8_t;       //! The type of a CoBo index
using asadid_type = int8_t;       //! The type of an AsAd index
using agetid_type = int8_t;       //! The type of an AGET index
using channelid_type = int8_t;    //! The type of a channel number
using padid_type = int16_t;       //! The type of a pad number

using timestamp_type = uint64_t;  //! The type of an event timestamp
using evtid_type = uint32_t;      //! The type of an event ID

}

#endif /* end of include guard: ATTPC_COMMON_TYPES_H */
