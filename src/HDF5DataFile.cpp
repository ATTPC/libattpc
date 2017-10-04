#include "attpc/common/HDF5DataFile.h"
#include "attpc/common/utilities.h"

namespace {

using H5Mode = decltype(H5F_ACC_TRUNC);  //! An access mode in the HDF5 library
using attpc::common::narrow_cast;

//! Map the file access mode onto one of the options provided by HDF5.
H5Mode getH5AccessMode(const attpc::common::HDF5DataFile::Mode mode) {
    using Mode = attpc::common::HDF5DataFile::Mode;
    switch (mode) {
        case Mode::readonly:
            return H5F_ACC_RDONLY;
        case Mode::readwrite:
            return H5F_ACC_RDWR;
        case Mode::truncate:
            return H5F_ACC_TRUNC;
        case Mode::create:
            return H5F_ACC_EXCL;
    }
}

}

namespace attpc {
namespace common {

const Eigen::Index HDF5DataFile::numColsInEvent = 512 + 5;    // num TBs + (cobo, asad, aget, channel, pad)
const std::string HDF5DataFile::timestampAttrName = "timestamp";
const H5::PredType HDF5DataFile::arrayH5DataType = H5::PredType::STD_I16LE;          // signed, 16-bit, little-endian integer
const H5::PredType HDF5DataFile::timestampAttrH5DataType = H5::PredType::STD_U64LE;  // unsigned, 64-bit, little-endian integer

HDF5DataFile::HDF5DataFile(const std::string& filename, const Mode mode)
{
    H5Mode hmode = getH5AccessMode(mode);
    file = H5::H5File(filename, hmode);
}

void HDF5DataFile::close() {
    file.close();
}

boost::optional<FullTraceEvent> HDF5DataFile::read(const evtid_type eventId, const std::string& groupName) {
    H5::Group group = file.openGroup(groupName);

    const std::string datasetName = std::to_string(eventId);
    H5::DataSet dataset;
    if (group.exists(datasetName)) {
        dataset = group.openDataSet(datasetName);
    }
    else {
        return boost::none;
    }

    H5::DataSpace dataspace = dataset.getSpace();
    if (dataspace.getSimpleExtentNdims() != 2) {
        throw BadData("Invalid number of dimensions in event read from HDF5 file.");
    }
    hsize_t dims[2];  // Dataset dimensions: [number of rows, number of columns]
    dataspace.getSimpleExtentDims(&dims[0]);

    if (dims[1] != numColsInEvent) {
        throw BadData("Event in HDF5 file had incorrect number of columns. Should be 517.");
    }

    EncodedEventArrayType data {dims[0], dims[1]};
    dataset.read(data.data(), arrayH5DataType);  // Read directly into Eigen array

    H5::Attribute tsAttr = dataset.openAttribute(timestampAttrName);
    timestamp_type timestamp;
    tsAttr.read(timestampAttrH5DataType, &timestamp);

    FullTraceEvent event = decodeEvent(data);
    event.setEventId(eventId);
    event.setTimestamp(timestamp);

    return event;
}

void HDF5DataFile::write(const FullTraceEvent& event, const std::string& groupName) {
    EncodedEventArrayType data = encodeEvent(event);

    const int dataspaceRank = 2;
    const hsize_t dataspaceDims[2] = {narrow_cast<hsize_t>(data.rows()), narrow_cast<hsize_t>(data.cols())};
    H5::DataSpace dataspace (dataspaceRank, dataspaceDims);

    H5::Group group = openOrCreateGroup(groupName);

    const std::string datasetName = std::to_string(event.getEventId());

    // Set up properties of the dataset (chunking, compression, filters)
    H5::DSetCreatPropList cparams;
    hsize_t chunkDims[2] = {1, numColsInEvent};  // Chunk size == one row
    cparams.setChunk(2, chunkDims);  // Chunked storage is required for compression
    cparams.setShuffle();   // The shuffle filter can enhance the compression ratio
    cparams.setDeflate(4);  // Compress the data with GZIP at level 4

    H5::DataSet dataset = group.createDataSet(datasetName, arrayH5DataType, dataspace, cparams);
    dataset.write(data.data(), arrayH5DataType);

    H5::DataSpace tsAttrDs {H5S_SCALAR};
    H5::Attribute tsAttr = dataset.createAttribute(timestampAttrName, timestampAttrH5DataType, tsAttrDs);
    timestamp_type timestamp = event.getTimestamp();
    tsAttr.write(timestampAttrH5DataType, &timestamp);
}

auto HDF5DataFile::encodeEvent(const FullTraceEvent& event) -> EncodedEventArrayType {
    const Eigen::Index numRows = narrow_cast<Eigen::Index>(event.numTraces());

    // The data must be put in a row-major array since HDF5 writes data in a row-major format
    EncodedEventArrayType array {numRows, numColsInEvent};

    for (auto iter = event.cbegin(); iter != event.cend(); ++iter) {
        Eigen::Index rowNum = std::distance(event.cbegin(), iter);
        const auto& traceAddr = iter->getHardwareAddress();
        const boost::optional<padid_type>& tracePad = iter->getPad();
        const Trace::ArrayType& traceData = iter->getData();

        // Need to map the column vector onto a row vector for the copy below
        const Eigen::Map<const Eigen::Array<Trace::ScalarType, 1, 512>> traceDataMap {traceData.data(), 512};

        array(rowNum, 0) = traceAddr.cobo;
        array(rowNum, 1) = traceAddr.asad;
        array(rowNum, 2) = traceAddr.aget;
        array(rowNum, 3) = traceAddr.channel;
        array(rowNum, 4) = tracePad.value_or(-1);
        array.block(rowNum, 5, 1, numColsInEvent - 5) = traceDataMap;
    }

    return array;
}

FullTraceEvent HDF5DataFile::decodeEvent(const Eigen::Ref<const EncodedEventArrayType>& array) {
    FullTraceEvent event {};
    for (Eigen::Index rowNum = 0; rowNum < array.rows(); ++rowNum) {
        const coboid_type    cobo    = narrow_cast<coboid_type>   (array(rowNum, 0));
        const asadid_type    asad    = narrow_cast<asadid_type>   (array(rowNum, 1));
        const agetid_type    aget    = narrow_cast<agetid_type>   (array(rowNum, 2));
        const channelid_type channel = narrow_cast<channelid_type>(array(rowNum, 3));
        const padid_type     pad     = narrow_cast<padid_type>    (array(rowNum, 4));
        const HardwareAddress addr {cobo, asad, aget, channel};

        Trace tr {addr, pad, array.block<1, numColsInEvent - 5>(rowNum, 5)};

        bool wasInserted;
        std::tie(std::ignore, wasInserted) = event.insertTrace(std::move(tr));
        if (!wasInserted) {
            throw BadData("Traces with duplicate hardware addresses present in event.");
        }
    }

    return event;
}

H5::Group HDF5DataFile::openOrCreateGroup(const std::string& groupName) {
    if (file.exists(groupName)) {
        return file.openGroup(groupName);
    }
    else {
        return file.createGroup(groupName);
    }
}

}
}
