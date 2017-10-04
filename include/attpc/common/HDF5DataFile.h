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

/**
 * @brief An HDF5-based data store for FullTraceEvent objects
 *
 * This class manages an HDF5 file and can read/write FullTraceEvent instances from/to it.
 *
 * Events are stored in the HDF5 group identified by the `groupName` argument to the read and write functions. This
 * group is called "get" by default.
 *
 * Events are stored as 2D datasets. Each row represents a single channel in the event, and the columns have the
 * following meanings:
 *
 *     cobo, asad, aget, channel, pad, data[0], data[1], ..., data[511]
 *
 * Therefore, each array has dimension Nx517, where N is the number of channels in the event. Missing pad numbers
 * are indicated with the value -1.
 *
 * The data is stored using a signed, little-endian, 16-bit integer data type. The samples from the GET system are
 * 12-bit unsigned integers, so a 16-bit signed integer will comfortably store all possible values, even if pedestal
 * subtraction is performed (thereby generating some negative values). Little-endian values are used since that is the
 * default endianness on most computers.
 *
 * The event ID is encoded in the name of the dataset. For example, the event with ID 45 would be stored at the
 * path "/get/45" in the HDF5 file (assuming the default name for the HDF5 group). The timestamp is stored in
 * an HDF5 attribute called "timestamp" attached to the dataset. Other values could be stored as attributes in the
 * future, if desired.
 *
 * @warning HDF5 files must be properly closed to prevent data corruption! This class will close the file automatically
 * upon destruction, but you must make sure that the class is, in fact, destroyed when the program ends. This might
 * not happen if the program is killed by a signal instead of terminating on its own. This can be addressed using a
 * signal handler to catch signals like SIGINT (and perhaps SIGTERM and SIGQUIT) and terminate gracefully.
 */
class HDF5DataFile {
public:
    /**
     * @brief The open mode for the file.
     *
     * These correspond to some of the open modes offered by the HDF5 library. These modes mean the following:
     * - `create`: Create a new file and open it in read/write mode. Fails if the file already exists.
     * - `truncate`: Delete the contents of the file if it already exists, and open in read/write mode.
     * - `readonly`: Open the file in read-only mode, failing if it does not exist.
     * - `readwrite`: Open the file in read/write mode, failing if it does not exist.
     */
    enum class Mode { create, truncate, readonly, readwrite };

    //! The type of the array that an event is encoded into.
    //! It is **essential** that this be row-major since that is the convention used by HDF5.
    using EncodedEventArrayType = Eigen::Array<Trace::ScalarType, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    /**
     * @brief Construct an instance and open a file
     * @param filename The path to the file.
     * @param mode     The open mode for the file.
     */
    HDF5DataFile(const std::string& filename, const Mode mode = Mode::readonly);

    /**
     * @brief Close the open file.
     *
     * The destructor will also do this automatically.
     */
    void close();

    /**
     * @brief Read an event from the file.
     *
     * If the file does not contain an event with the given event ID, then boost::none will be returned instead.
     *
     * @param  eventId   The event ID of the event to read.
     * @param  groupName The name of the HDF5 group in which the event is stored.
     * @return           The event, if it exists; otherwise, boost::none.
     * @throws BadData   If the event in the file had incorrect dimensions or was otherwise invalid.
     */
    boost::optional<FullTraceEvent> read(const evtid_type eventId, const std::string& groupName = "get");

    /**
     * @brief Write an event to the file.
     *
     * This will fail if the event already exists in the file.
     *
     * @param event     The event to write.
     * @param groupName The name of the HDF5 group where the event should be written.
     */
    void write(const FullTraceEvent& event, const std::string& groupName = "get");

    /**
     * @brief Encode an event into the format stored in the file.
     *
     * This is mainly for internal use.
     *
     * @param  event The event to encode.
     * @return       The event as an array. The format is described in the class documentation.
     */
    static EncodedEventArrayType encodeEvent(const FullTraceEvent& event);

    /**
     * @brief Decode an event from the array format used in the file.
     *
     * This is mainly for internal use.
     *
     * @param  array   The event as an array. The format is described in the class documentation.
     * @return         The decoded event.
     * @throws BadData If Trace insertion into the decoded event fails. This is usually caused by multiple rows
     *                 in the array having the same CoBo, AsAd, AGET, and channel.
     */
    static FullTraceEvent decodeEvent(const Eigen::Ref<const EncodedEventArrayType>& array);

    //! Indicates that the data read from the file did not represent a valid event.
    class BadData : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
    };

protected:
    /**
     * @brief Opens the HDF5 group, or creates it if it doesn't exist already.
     * @param  groupName The name of the group.
     * @return           The group object.
     */
    H5::Group openOrCreateGroup(const std::string& groupName);

private:
    //! The file object
    H5::H5File file;

    //! The number of columns in a valid event
    static const Eigen::Index numColsInEvent;
    //! The name of the timestamp attribute attached to each event dataset
    static const std::string timestampAttrName;
    //! The scalar data type used to represent samples in the HDF5 file.
    //! This is a signed, 16-bit, little-endian integer.
    static const H5::PredType arrayH5DataType;
    //! The data type used to store the timestamp in the HDF5 file.
    //! This is an unsigned, 64-bit, little-endian integer.
    static const H5::PredType timestampAttrH5DataType;
};

}
}

#endif /* end of include guard: ATTPC_COMMON_HDF5DATAFILE_H */
