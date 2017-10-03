#ifndef ATTPC_COMMON_HARDWAREADDRESS_H
#define ATTPC_COMMON_HARDWAREADDRESS_H

#include "attpc/common/types.h"
#include "attpc/common/utilities.h"
#include <tuple>
#include <cassert>
#include <functional>

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

template <>
struct std::hash<attpc::common::HardwareAddress> {
    size_t operator()(const attpc::common::HardwareAddress& addr) const {
        assert(addr.cobo > 0 && addr.asad > 0 && addr.aget > 0 && addr.channel > 0);
        uint64_t cobo = static_cast<uint64_t>(addr.cobo);
        uint64_t asad = static_cast<uint64_t>(addr.asad);
        uint64_t aget = static_cast<uint64_t>(addr.aget);
        uint64_t channel = static_cast<uint64_t>(addr.channel);

        uint64_t combined = channel | (aget << 8) | (asad << 16) | (cobo << 24);

        return std::hash<uint64_t>()(combined);
    }
};

#endif /* end of include guard: ATTPC_COMMON_HARDWAREADDRESS_H */
