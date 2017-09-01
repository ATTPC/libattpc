#ifndef ATTPC_COMMON_HDF5DATAFILE_H
#define ATTPC_COMMON_HDF5DATAFILE_H

#include "types.h"
#include "FullTraceEvent.h"
#include <string>
#include <H5Cpp.h>
#include <boost/optional.hpp>
#include <stdexcept>

namespace attpc {
namespace common {

class HDF5DataFile {
public:
    enum class Mode { create, truncate, readonly, readwrite };

    HDF5DataFile(const std::string& filename, const Mode mode = Mode::readonly);

    boost::optional<FullTraceEvent> read(const evtid_type eventId, const std::string& groupName = "get");
    void write(const FullTraceEvent& event, const std::string& groupName = "get");

    class BadData : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

protected:
    H5::Group openOrCreateGroup(const std::string& groupName);

private:
    H5::H5File file;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_HDF5DATAFILE_H */
