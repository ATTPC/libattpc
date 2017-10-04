#ifndef ATTPC_COMMON_HDF5DATAFILE_H
#define ATTPC_COMMON_HDF5DATAFILE_H

#include "attpc/common/types.h"
#include "attpc/common/FullTraceEvent.h"
#include <string>
#include <H5Cpp.h>
#include <boost/optional.hpp>
#include <stdexcept>

namespace attpc {
namespace common {

class HDF5DataFile {
public:
    enum class Mode { create, truncate, readonly, readwrite };
    using EncodedEventArrayType = Eigen::Array<Trace::ScalarType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    HDF5DataFile(const std::string& filename, const Mode mode = Mode::readonly);

    void close();

    boost::optional<FullTraceEvent> read(const evtid_type eventId, const std::string& groupName = "get");
    void write(const FullTraceEvent& event, const std::string& groupName = "get");

    static EncodedEventArrayType encodeEvent(const FullTraceEvent& event);
    static FullTraceEvent decodeEvent(const Eigen::Ref<const EncodedEventArrayType>& array);

    class BadData : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

protected:
    H5::Group openOrCreateGroup(const std::string& groupName);

private:
    H5::H5File file;

    static const Eigen::Index numColsInEvent;
    static const std::string timestampAttrName;

    static const H5::PredType arrayH5DataType;          // signed, 16-bit, little-endian integer
    static const H5::PredType timestampAttrH5DataType;  // unsigned, 64-bit, little-endian integer
};

}
}

#endif /* end of include guard: ATTPC_COMMON_HDF5DATAFILE_H */
