#ifndef ATTPC_COMMON_HARDWAREADDRESS_H
#define ATTPC_COMMON_HARDWAREADDRESS_H

#include "attpc/common/types.h"
#include "attpc/common/utilities.h"
#include <tuple>
#include <functional>
#include <ostream>

namespace attpc {
namespace common {

/**
 * @brief Represents the location of a channel in the GET electronics hierarchy.
 *
 * This struct wraps the CoBo, AsAd, AGET, and channel number associated with a given channel in the
 * GET electronics system. It can be used in both `std::map` and `std::unordered_map` containers
 * since it supports `<` and `==` operators (for sorting) and provides a hash function.
 *
 * Pad numbers are not included in this struct, and should be stored separately.
 */
struct HardwareAddress {
    coboid_type cobo;        //! The CoBo index
    asadid_type asad;        //! The AsAd index
    agetid_type aget;        //! The AGET index
    channelid_type channel;  //! The channel number

    /**
     * @brief Default-construct a HardwareAddress object
     *
     * This constructor sets all four indices to zero.
     */
    HardwareAddress();
    /**
     * @brief Construct a HardwareAddress object and set the values.
     * @param cobo_    The CoBo index
     * @param asad_    The AsAd index
     * @param aget_    The AGET index
     * @param channel_ The channel number
     */
    HardwareAddress(coboid_type cobo_, asadid_type asad_, agetid_type aget_, channelid_type channel_);

    /**
     * @brief Compare two HardwareAddresses for equality
     * @return True if all four indices are equal.
     */
    bool operator==(const HardwareAddress& other) const;
    /**
     * @brief Determine if one HardwareAddress is less than another.
     *
     * The comparison is done lexicographically. In other words, if a list of all channels were to be sorted
     * using this comparison operator, the order would be something like
     *
     *     {0, 0, 0, 0}, {0, 0, 0, 1}, ..., {0, 0, 0, 67}, {0, 0, 1, 0}, ..., {0, 0, 3, 67}, {0, 1, 0, 0}, ...
     *
     * So, all channels for CoBo 0 come before all channels from CoBo 1, etc.
     *
     * @return True if the left-hand side is less than the right-hand side
     */
    bool operator<(const HardwareAddress& other) const;
};

/**
 * @brief Insert the address into the given output stream.
 *
 * This allows the HardwareAddress to be printed. It is displayed as
 *
 *     {[cobo], [asad], [aget], [channel]}
 *
 * where the values in square brackets are replaced with the appropriate numbers.
 *
 * @return The output stream.
 */
std::ostream& operator<<(std::ostream& os, const HardwareAddress& addr);

}
}

/**
 * @brief Specialization of `std::hash` for HardwareAddress objects.
 *
 * This hashes the `HardwareAddress` so it can be used as the key in a `std::unordered_map`.
 */
template <>
struct std::hash<attpc::common::HardwareAddress> {
    size_t operator()(const attpc::common::HardwareAddress& addr) const;
};

#endif /* end of include guard: ATTPC_COMMON_HARDWAREADDRESS_H */
