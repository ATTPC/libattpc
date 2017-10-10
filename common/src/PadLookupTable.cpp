#include "attpc/common/PadLookupTable.h"
#include <cassert>
#include <fstream>
#include <sstream>

namespace {
    // Implementation of the find function. This is separate since it's the same for both the forward and
    // reverse cases, and only the types vary.
    template <class MapType>
    boost::optional<typename MapType::mapped_type>
    findImpl(const MapType& map, const typename MapType::key_type& key) {
        auto foundItem = map.find(key);
        if (foundItem != map.end()) {
            return foundItem->second;
        }
        else {
            return boost::none;
        }
    }
}

namespace attpc {
namespace common {

PadLookupTable::PadLookupTable(const std::string& path) {
    std::ifstream file {path};

    std::string line;
    while (getline(file, line, '\n')) {
        HardwareAddress addr;
        padid_type pad;

        std::stringstream lineStream {line};
        std::string element;

        getline(lineStream, element, ',');
        if (element == "-1" || element == "") { continue; }
        addr.cobo = std::stoi(element);

        getline(lineStream, element, ',');
        addr.asad = std::stoi(element);

        getline(lineStream, element, ',');
        addr.aget = std::stoi(element);

        getline(lineStream, element, ',');
        addr.channel = std::stoi(element);

        getline(lineStream, element, ',');
        pad = std::stoi(element);

        insert(addr, pad);
    }
}

void PadLookupTable::insert(const HardwareAddress& addr, padid_type pad) {
    forwardTable[addr] = pad;
    reverseTable[pad] = addr;
    assert(forwardTable.size() == reverseTable.size());
}

boost::optional<padid_type> PadLookupTable::find(const HardwareAddress& addr) const {
    return findImpl(forwardTable, addr);  // See implementation above
}

boost::optional<HardwareAddress> PadLookupTable::reverseFind(padid_type pad) const {
    return findImpl(reverseTable, pad);  // See implementation above
}

size_t PadLookupTable::size() const {
    assert(forwardTable.size() == reverseTable.size());
    return forwardTable.size();
}

bool PadLookupTable::empty() const {
    return size() == 0;
}

}
}
