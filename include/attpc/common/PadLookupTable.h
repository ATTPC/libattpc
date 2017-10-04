#ifndef ATTPC_COMMON_PADLOOKUPTABLE_H
#define ATTPC_COMMON_PADLOOKUPTABLE_H

#include "attpc/common/HardwareAddress.h"
#include "attpc/common/types.h"
#include <map>
#include <boost/optional.hpp>

namespace attpc {
namespace common {

/**
 * @brief Maps HardwareAddress values to pad numbers.
 *
 * This can be used to set the pad number for each Trace based on its CoBo, AsAd, AGET, and channel number.
 * The pad map can be read from a CSV file with the columns
 *
 *     cobo,asad,aget,channel,pad
 *
 * The CSV file should have standard UNIX line endings, and it must not have any column headings. Unmapped
 * pads can be represented in the file with the value -1 for the CoBo, AsAd, AGET, and channel.
 */
class PadLookupTable {
public:
    //! Construct an empty PadLookupTable
    PadLookupTable() = default;
    /**
     * @brief Read the lookup table from the file at the given path.
     *
     * The file should be a CSV file with the format given above in the class documentation.
     *
     * @param path Path to the CSV file.
     */
    PadLookupTable(const std::string& path);

    /**
     * @brief Insert an entry into the lookup table.
     * @param addr The HardwareAddress
     * @param pad  The pad number
     */
    void insert(const HardwareAddress& addr, padid_type pad);

    /**
     * @brief Find the pad number associated with a HardwareAddress.
     *
     * This returns boost::none if the HardwareAddress is not found in the lookup table.
     *
     * @param  addr The HardwareAddress
     * @return      The pad number
     */
    boost::optional<padid_type> find(const HardwareAddress& addr) const;
    /**
     * @brief Find the HardwareAddress associated with a pad number.
     *
     * This returns boost::none if the pad number is not found in the lookup table.
     *
     * @param  pad The pad number
     * @return     The HardwareAddress
     */
    boost::optional<HardwareAddress> reverseFind(padid_type pad) const;

    //! Get the number of mapped pads
    size_t size() const;
    //! Returns true if there are no mapped pads
    bool empty() const;

private:
    // These two maps must be kept in sync
    std::map<HardwareAddress, padid_type> forwardTable;
    std::map<padid_type, HardwareAddress> reverseTable;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_PADLOOKUPTABLE_H */
