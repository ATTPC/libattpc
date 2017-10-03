#ifndef ATTPC_COMMON_HARDWAREADDRESS_H
#define ATTPC_COMMON_HARDWAREADDRESS_H

#include "attpc/common/types.h"
#include "attpc/common/utilities.h"
#include <tuple>
#include <cassert>
#include <functional>
#include <ostream>

namespace attpc {
namespace common {

struct HardwareAddress {
    coboid_type cobo;
    asadid_type asad;
    agetid_type aget;
    channelid_type channel;

    HardwareAddress();
    HardwareAddress(coboid_type cobo_, asadid_type asad_, agetid_type aget_, channelid_type channel_);

    bool operator==(const HardwareAddress& other) const;
    bool operator<(const HardwareAddress& other) const;
};

std::ostream& operator<<(std::ostream& os, const HardwareAddress& addr);

}
}

template <>
struct std::hash<attpc::common::HardwareAddress> {
    size_t operator()(const attpc::common::HardwareAddress& addr) const;
};

#endif /* end of include guard: ATTPC_COMMON_HARDWAREADDRESS_H */
