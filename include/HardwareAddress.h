#ifndef ATTPC_COMMON_HARDWAREADDRESS_H
#define ATTPC_COMMON_HARDWAREADDRESS_H

#include "types.h"
#include <tuple>

namespace attpc {
namespace common {

struct HardwareAddress {
    coboid_type cobo;
    asadid_type asad;
    agetid_type aget;
    channelid_type channel;

    HardwareAddress()
    : cobo(0), asad(0), aget(0), channel(0) {}

    HardwareAddress(coboid_type cobo_, asadid_type asad_, agetid_type aget_, channelid_type channel_)
    : cobo(cobo_), asad(asad_), aget(aget_), channel(channel_) {}

    bool operator==(const HardwareAddress& other) const {
        return (cobo == other.cobo)
            && (asad == other.asad)
            && (aget == other.aget)
            && (channel == other.channel);
    }

    bool operator<(const HardwareAddress& other) const {
        return std::tie(cobo, asad, aget, channel) < std::tie(other.cobo, other.asad, other.aget, other.channel);
    }
};

}
}

#endif /* end of include guard: ATTPC_COMMON_HARDWAREADDRESS_H */
