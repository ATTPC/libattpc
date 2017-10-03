#ifndef ATTPC_COMMON_PADLOOKUPTABLE_H
#define ATTPC_COMMON_PADLOOKUPTABLE_H

#include "attpc/common/HardwareAddress.h"
#include "attpc/common/types.h"
#include <map>
#include <boost/optional.hpp>

namespace attpc {
namespace common {

class PadLookupTable {
public:
    PadLookupTable() = default;
    PadLookupTable(const std::string& path);

    void insert(const HardwareAddress& addr, padid_type pad);

    boost::optional<padid_type> find(const HardwareAddress& addr) const;
    boost::optional<HardwareAddress> reverseFind(padid_type pad) const;

    size_t size() const;
    bool empty() const;

private:
    std::map<HardwareAddress, padid_type> forwardTable;
    std::map<padid_type, HardwareAddress> reverseTable;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_PADLOOKUPTABLE_H */
