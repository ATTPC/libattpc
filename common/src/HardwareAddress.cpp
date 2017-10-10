#include "attpc/common/HardwareAddress.h"
#include <cassert>

namespace attpc {
namespace common {

HardwareAddress::HardwareAddress()
: cobo(0), asad(0), aget(0), channel(0) {}

HardwareAddress::HardwareAddress(coboid_type cobo_, asadid_type asad_, agetid_type aget_, channelid_type channel_)
: cobo(cobo_), asad(asad_), aget(aget_), channel(channel_) {}

bool HardwareAddress::operator==(const HardwareAddress& other) const {
    return (cobo == other.cobo)
        && (asad == other.asad)
        && (aget == other.aget)
        && (channel == other.channel);
}

bool HardwareAddress::operator<(const HardwareAddress& other) const {
    // std::tie creates a std::tuple, and tuple objects are lexicographically ordered.
    return std::tie(cobo, asad, aget, channel) < std::tie(other.cobo, other.asad, other.aget, other.channel);
}

std::ostream& operator<<(std::ostream& os, const HardwareAddress& addr) {
    os << "{" << addr.cobo << ", " << addr.asad << ", " << addr.aget << ", " << addr.channel << "}";
    return os;
}

}
}

size_t std::hash<attpc::common::HardwareAddress>::operator()(const attpc::common::HardwareAddress& addr) const {
    // NOTE: The values *must* be converted to unsigned integers before left-shifting to avoid
    // potentially undefined behavior. This implementation works since the four values are stored
    // in 8-bit integers currently. If this is changed, this hash function might not perform well.
    assert(addr.cobo >= 0 && addr.asad >= 0 && addr.aget >= 0 && addr.channel >= 0);
    uint64_t cobo = static_cast<uint64_t>(addr.cobo);
    uint64_t asad = static_cast<uint64_t>(addr.asad);
    uint64_t aget = static_cast<uint64_t>(addr.aget);
    uint64_t channel = static_cast<uint64_t>(addr.channel);

    uint64_t combined = channel | (aget << 8) | (asad << 16) | (cobo << 24);

    return std::hash<uint64_t>()(combined);
}
